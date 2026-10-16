'use strict';

const $ = (id) => document.getElementById(id);

let ws = null;
let state = null;

function load(key, fallback) {
  try {
    const v = localStorage.getItem(key);
    return v === null ? fallback : JSON.parse(v);
  } catch (e) {
    return fallback;
  }
}

function save(key, value) {
  try {
    localStorage.setItem(key, JSON.stringify(value));
  } catch (e) { /* private mode: not remembered, still works */ }
}

// ======================= connection =======================

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => $('offline').classList.add('hidden');
  ws.onclose = () => {
    $('offline').classList.remove('hidden');
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.t === 'state') {
      state = msg;
      render();
    } else if (msg.t === 'event') {
      toast(msg.text, msg.level);
    }
  };
}

function send(cmd, args = {}, quiet = false) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ cmd, ...args }));
    return true;
  }
  if (!quiet) toast('Robotla əlaqə yoxdur', 'error');
  return false;
}

function toast(text, level = 'info') {
  const el = document.createElement('div');
  el.className = `toast ${level}`;
  el.textContent = text;
  $('toasts').appendChild(el);
  setTimeout(() => el.remove(), 5000);
}

// ======================= dialogs =======================
// In-page replacements for confirm()/prompt(), which some phone browsers
// (in-app browsers of QR scanner apps) silently answer with "cancel".

let modalResolve = null;

function openModal(text, { okLabel = 'Bəli', danger = false, input = null } = {}) {
  return new Promise((resolve) => {
    if (modalResolve) modalResolve(null);
    modalResolve = resolve;
    $('modalText').textContent = text;
    const inp = $('modalInput');
    inp.classList.toggle('hidden', input === null);
    if (input !== null) {
      inp.value = '';
      inp.placeholder = input;
    }
    const ok = $('modalOk');
    ok.textContent = okLabel;
    ok.className = danger ? 'danger' : 'primary';
    $('modal').classList.remove('hidden');
    if (input !== null) setTimeout(() => inp.focus(), 50);
  });
}

function closeModal(value) {
  $('modal').classList.add('hidden');
  const resolve = modalResolve;
  modalResolve = null;
  if (resolve) resolve(value);
}

$('modalOk').addEventListener('click', () => {
  const inp = $('modalInput');
  closeModal(inp.classList.contains('hidden') ? true : inp.value.trim());
});
$('modalCancel').addEventListener('click', () => closeModal(null));
$('modal').addEventListener('click', (e) => { if (e.target === $('modal')) closeModal(null); });
$('modalInput').addEventListener('keydown', (e) => { if (e.key === 'Enter') $('modalOk').click(); });

const ask = async (text, opts) => (await openModal(text, opts)) === true;
const askText = (text, placeholder) => openModal(text, { okLabel: 'Saxla', input: placeholder });

// ======================= tabs =======================

function showTab(name) {
  document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t.id === `tab-${name}`));
  document.querySelectorAll('.tabbar button').forEach((b) => b.classList.toggle('on', b.dataset.tab === name));
  if (name === 'map') resizeCanvas();
  camLoop();
}

document.querySelectorAll('.tabbar button').forEach((b) => {
  b.addEventListener('click', () => showTab(b.dataset.tab));
});

document.querySelectorAll('[data-goto-tab]').forEach((b) => {
  b.addEventListener('click', () => {
    showTab(b.dataset.gotoTab);
    if (b.dataset.tool) setTool(b.dataset.tool);
  });
});

// ======================= status rendering =======================

const MODE_NAMES = { navigation: 'Naviqasiya', mapping: 'Xəritələmə', idle: 'Gözləmə' };
const MODE_STATES = { ready: 'hazır', starting: 'başladılır…', waiting: 'motor və lidar gözlənilir…',
                      localizing: 'mövqe yoxlanılır…', error: 'xəta', idle: '' };

function exploreText(s) {
  if (s.explore_pending) return 'xəritələmə hazır olan kimi başlayacaq…';
  const status = s.explorer.on ? s.explorer.status : '';
  if (!status) return 'dayanıb';
  if (status === 'starting' || status === 'waiting for navigation') return 'başlayır…';
  if (status === 'looking around') return 'ətrafa baxır (yerində dönür)';
  if (status === 'returning home') return 'başlanğıca qayıdır';
  if (status === 'done') return 'bitdi';
  const m = status.match(/exploring \((\d+)/);
  if (m) return `kəşf edir (${m[1]} açıq sərhəd)`;
  if (status === 'exploring') return 'kəşf edir';
  return status;
}

const fmt = (v) => v.toFixed(2);
const deg = (rad) => Math.round((rad * 180) / Math.PI);

let lastTablesJson = '';
let lastMapsJson = '';

function render() {
  const s = state;
  const pill = $('modePill');
  const stateText = MODE_STATES[s.mode_state] || '';
  pill.textContent = `${MODE_NAMES[s.mode] || s.mode}${stateText ? ' · ' + stateText : ''}`;
  pill.className = 'pill ' + ({ ready: 'ok', starting: 'warn', waiting: 'warn', localizing: 'warn',
                                 error: 'bad' }[s.mode_state] || '');

  $('sEsp').classList.toggle('on', s.sensors.esp32);
  $('sLidar').classList.toggle('on', s.sensors.lidar);
  $('sCam').classList.toggle('on', s.sensors.camera);

  renderBattery(s);

  const locBad = s.mode === 'navigation' && s.mode_state === 'ready' &&
    (!s.loc_confirmed || (s.loc && s.loc.std_xy > 0.35));
  $('locWarn').classList.toggle('hidden', !locBad);

  const nav = s.nav;
  $('navBar').classList.toggle('hidden', !nav.active);
  if (nav.active) {
    let detail = '';
    if ((nav.kind === 'goto' || nav.kind === 'home') && nav.remaining != null) {
      detail = ` — ${nav.remaining.toFixed(1)} m qaldı`;
    }
    if (nav.kind === 'route') detail = ` — nöqtə ${Math.min(nav.current + 1, nav.total)}/${nav.total}`;
    $('navText').textContent = nav.text + detail;
  }
  $('homeBtn').classList.toggle('on', nav.active && nav.kind === 'home');

  $('velText').textContent = `${s.velocity[0].toFixed(2)} m/s · ${s.velocity[1].toFixed(2)} rad/s`;
  syncSlider('navSpeed', 'navVal', s.speed.nav, s.speed.nav_range, mps);
  syncSlider('manualSpeed', 'manualVal', s.speed.manual, s.speed.manual_range, mps);
  syncSlider('autoHomeLevel', 'autoHomeVal', s.auto_home.level, [10, 60], (v) => `${v}%`);
  if (!sliderBusy.autoHome) $('autoHome').checked = s.auto_home.on;

  const home = s.home;
  $('homeText').textContent = home
    ? `x ${fmt(home.x)}, y ${fmt(home.y)}, ${deg(home.yaw)}°`
    : (s.mode === 'mapping' ? 'xəritələmə zamanı istifadə olunmur' : 'təyin edilməyib');

  // mapping tab
  const mapping = s.mode === 'mapping';
  const mappingReady = mapping && s.mode_state === 'ready';
  $('mapStartCard').classList.toggle('hidden', mapping);
  $('mappingCard').classList.toggle('hidden', !mapping);
  $('mappingState').textContent = stateText || '—';
  $('exploreText').textContent = exploreText(s);
  $('exploreOn').disabled = !mappingReady || s.explorer.on || s.explore_pending;
  $('exploreOff').disabled = !s.explorer.on && !s.explore_pending;
  $('saveMap').disabled = !mappingReady;
  $('mapName').disabled = !mappingReady;
  $('modeText').textContent = MODE_NAMES[s.mode] + (stateText ? ` (${stateText})` : '');

  $('locText').textContent = s.loc
    ? `±${s.loc.std_xy.toFixed(2)} m, ±${s.loc.std_yaw.toFixed(0)}°${s.loc_confirmed ? '' : ' (təsdiqlənməyib)'}`
    : (s.mode === 'navigation' ? 'gözlənilir' : 'naviqasiya rejimində deyil');
  $('activeMapText').textContent = s.active_map || '—';
  $('robotPending').classList.toggle('hidden', !(s.robot && s.robot.pending_restart));
  renderPerception(s);

  const tablesJson = JSON.stringify(s.tables);
  if (tablesJson !== lastTablesJson) {
    lastTablesJson = tablesJson;
    renderTables();
  }
  const mapsJson = JSON.stringify([s.maps, s.active_map]);
  if (mapsJson !== lastMapsJson) {
    lastMapsJson = mapsJson;
    renderMaps();
  }

  if (view.follow && s.pose) {
    view.cx = s.pose.x;
    view.cy = s.pose.y;
  }
  maybeLoadMap();
  requestDraw();
}

function renderBattery(s) {
  const b = s.battery;
  $('battery').classList.toggle('hidden', !b);
  if (!b) {
    $('battDetail').textContent = 'tapılmadı';
    return;
  }
  $('battText').textContent = `${b.percent}%`;
  const fill = $('battFill');
  fill.style.width = `${Math.max(6, Math.min(100, b.percent))}%`;
  let cls = '';
  if (b.plugged) cls = 'chg';
  else if (b.percent <= s.auto_home.level) cls = 'low';
  else if (b.percent <= s.auto_home.level + 15) cls = 'mid';
  fill.className = `batt-fill ${cls}`;
  $('battery').classList.toggle('plugged', b.plugged);
  $('battDetail').textContent = `${b.percent}% · ${b.plugged ? 'şarjdadır' : 'batareyadan işləyir'}`;
}

// ======================= sliders =======================

const sliderBusy = {};

function syncSlider(id, labelId, value, range, label) {
  const el = $(id);
  el.min = range[0];
  el.max = range[1];
  if (!sliderBusy[id]) el.value = value;
  $(labelId).textContent = label(Number(el.value));
}

function bindSlider(id, labelId, onChange, label) {
  const el = $(id);
  el.addEventListener('input', () => {
    sliderBusy[id] = true;
    $(labelId).textContent = label(Number(el.value));
  });
  el.addEventListener('change', () => {
    onChange(Number(el.value));
    setTimeout(() => { sliderBusy[id] = false; }, 800);
  });
}
const mps = (v) => `${v.toFixed(2)} m/s · ${(v * 3.6).toFixed(1)} km/saat`;
bindSlider('navSpeed', 'navVal', (v) => send('speed', { nav: v }), mps);
bindSlider('manualSpeed', 'manualVal', (v) => send('speed', { manual: v }), mps);
bindSlider('autoHomeLevel', 'autoHomeVal', (v) => send('auto_home', { level: v }), (v) => `${v}%`);

// ======================= map canvas =======================

const canvas = $('map');
const ctx = canvas.getContext('2d');
const view = { scale: 50, cx: 0, cy: 0, follow: true };
let cw = 0;
let ch = 0;
let mapImg = null;
let mapMeta = null;
let mapVersion = -1;
let mapLoading = false;
let lastMapLoad = 0;
let drawQueued = false;
let showScan = load('showScan', true);

let tool = 'view';
let routePoints = [];
let dragPreview = null; // {x, y, yaw} while placing with a drag

function resizeCanvas() {
  const r = canvas.getBoundingClientRect();
  if (!r.width) return;
  const dpr = window.devicePixelRatio || 1;
  cw = r.width;
  ch = r.height;
  canvas.width = Math.round(cw * dpr);
  canvas.height = Math.round(ch * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  requestDraw();
}
window.addEventListener('resize', resizeCanvas);
// Banners appearing/disappearing change the map area without a window resize.
new ResizeObserver(resizeCanvas).observe(canvas);

const w2s = (x, y) => [cw / 2 + (x - view.cx) * view.scale, ch / 2 - (y - view.cy) * view.scale];
const s2w = (px, py) => [view.cx + (px - cw / 2) / view.scale, view.cy - (py - ch / 2) / view.scale];

function maybeLoadMap() {
  const m = state && state.map;
  if (!m) {
    mapImg = null;
    mapMeta = null;
    mapVersion = -1;
    return;
  }
  if (m.version === mapVersion || mapLoading) return;
  // SLAM republishes the map every few seconds; no need to refetch faster.
  if (mapImg && Date.now() - lastMapLoad < 2000) return;
  mapLoading = true;
  const img = new Image();
  img.onload = () => {
    const first = !mapImg;
    mapImg = img;
    mapMeta = m;
    mapVersion = m.version;
    mapLoading = false;
    lastMapLoad = Date.now();
    if (first && !(view.follow && state.pose)) fitMap();
    requestDraw();
  };
  img.onerror = () => {
    mapLoading = false;
    lastMapLoad = Date.now();
  };
  img.src = `/api/map.png?v=${m.version}`;
}

function fitMap() {
  if (!mapMeta || !cw) return;
  const wm = mapMeta.width * mapMeta.resolution;
  const hm = mapMeta.height * mapMeta.resolution;
  view.scale = Math.min(cw / wm, ch / hm) * 0.95;
  view.cx = mapMeta.ox + wm / 2;
  view.cy = mapMeta.oy + hm / 2;
}

function requestDraw() {
  if (drawQueued) return;
  drawQueued = true;
  requestAnimationFrame(() => {
    drawQueued = false;
    draw();
  });
}

function drawArrow(x, y, yaw, color, len = 0.35) {
  const [sx, sy] = w2s(x, y);
  const [ex, ey] = w2s(x + Math.cos(yaw) * len, y + Math.sin(yaw) * len);
  ctx.strokeStyle = color;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(sx, sy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
}

function drawMarker(x, y, color, label) {
  const [sx, sy] = w2s(x, y);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(sx, sy, 8, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2;
  ctx.stroke();
  if (label) {
    ctx.font = 'bold 13px system-ui';
    ctx.lineWidth = 3;
    ctx.strokeStyle = 'rgba(0,0,0,.75)';
    ctx.strokeText(label, sx + 11, sy - 9);
    ctx.fillStyle = '#fff';
    ctx.fillText(label, sx + 11, sy - 9);
  }
}

function drawPoints(flat, color) {
  if (!flat || !flat.length) return;
  ctx.fillStyle = color;
  const size = Math.max(2.5, Math.min(5, view.scale * 0.04));
  for (let i = 0; i < flat.length; i += 2) {
    const [sx, sy] = w2s(flat[i], flat[i + 1]);
    ctx.fillRect(sx - size / 2, sy - size / 2, size, size);
  }
}

// Footprint used until the robot profile arrives with the first state.
const FALLBACK_FOOTPRINT = [[0.225, 0.21], [0.225, -0.21], [-0.225, -0.21], [-0.225, 0.21]];

function drawRobot(p) {
  const [sx, sy] = w2s(p.x, p.y);
  const geo = state && state.robot;
  const fp = geo ? geo.footprint : FALLBACK_FOOTPRINT;
  const k = view.scale;
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-p.yaw);
  // Robot frame on the canvas: x forward -> +x, y left -> -y.
  if (geo) {
    const r = geo.wheel_radius;
    const hw = geo.wheel_width / 2;
    ctx.fillStyle = '#111827';
    for (const w of geo.wheels) ctx.fillRect((w.x - r) * k, (-w.y - hw) * k, 2 * r * k, 2 * hw * k);
  }
  ctx.beginPath();
  fp.forEach(([x, y], i) => (i ? ctx.lineTo(x * k, -y * k) : ctx.moveTo(x * k, -y * k)));
  ctx.closePath();
  ctx.fillStyle = 'rgba(59,130,246,.45)';
  ctx.strokeStyle = '#1d4ed8';
  ctx.lineWidth = 2;
  ctx.fill();
  ctx.stroke();
  const front = Math.max(...fp.map((q) => q[0])) * k;
  const half = Math.max(...fp.map((q) => q[1])) * k;
  const tip = Math.max(front, 10);
  const len = Math.max(front * 0.6, 8);
  ctx.fillStyle = '#fff';
  ctx.beginPath();
  ctx.moveTo(tip, 0);
  ctx.lineTo(tip - len, -Math.max(half * 0.3, 6));
  ctx.lineTo(tip - len, Math.max(half * 0.3, 6));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function draw() {
  if (!cw) return;
  ctx.fillStyle = '#cdcdcd';
  ctx.fillRect(0, 0, cw, ch);
  if (mapImg && mapMeta) {
    const res = mapMeta.resolution;
    const [x0, y0] = w2s(mapMeta.ox, mapMeta.oy + mapMeta.height * res);
    ctx.imageSmoothingEnabled = view.scale < 20;
    ctx.drawImage(mapImg, x0, y0, mapMeta.width * res * view.scale, mapMeta.height * res * view.scale);
  } else {
    ctx.fillStyle = '#334155';
    ctx.font = '15px system-ui';
    ctx.fillText('Xəritə yüklənir…', 16, ch / 2);
  }
  if (!state) return;

  if (showScan && state.scan) {
    drawPoints(state.scan.camera, '#0891b2');
    drawPoints(state.scan.lidar, '#dc2626');
  }
  drawObjects();

  for (const [name, t] of Object.entries(state.tables || {})) {
    drawArrow(t.x, t.y, t.yaw, '#b45309', 0.25);
    drawMarker(t.x, t.y, '#f59e0b', name);
  }

  if (state.home) {
    drawArrow(state.home.x, state.home.y, state.home.yaw, '#15803d', 0.3);
    drawMarker(state.home.x, state.home.y, '#16a34a', 'Ev');
  }

  if (routePoints.length) {
    ctx.strokeStyle = '#7c3aed';
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    routePoints.forEach((p, i) => {
      const [sx, sy] = w2s(p.x, p.y);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
    ctx.setLineDash([]);
    routePoints.forEach((p, i) => {
      if (p.yaw != null) drawArrow(p.x, p.y, p.yaw, '#7c3aed', 0.25);
      drawMarker(p.x, p.y, '#8b5cf6', String(i + 1));
    });
  }

  if (dragPreview) {
    drawMarker(dragPreview.x, dragPreview.y, '#16a34a');
    if (dragPreview.yaw != null) drawArrow(dragPreview.x, dragPreview.y, dragPreview.yaw, '#16a34a', 0.5);
  }

  if (state.pose) drawRobot(state.pose);
}

// ---------- tools ----------

const HINTS = {
  view: '',
  goto: 'Toxunun: robot ora gedəcək. Barmağı sürüşdürsəniz son istiqaməti də verirsiniz.',
  route: 'Nöqtələrə ardıcıl toxunun, sonra aşağıda "Başlat".',
  table: 'Masanın yerinə toxunun (sürüşdürərək istiqamət verin), sonra ad yazın.',
  home: 'Şarj yerinə toxunun və robotun orada baxmalı olduğu tərəfə sürüşdürün.',
  pose: 'Robotun həqiqi yerinə toxunub baxdığı tərəfə sürüşdürün.',
};

function setTool(name) {
  tool = name;
  document.querySelectorAll('#mapTools button').forEach((b) => b.classList.toggle('on', b.dataset.tool === name));
  $('mapHint').textContent = HINTS[name];
  $('routeBar').classList.toggle('hidden', name !== 'route');
  resizeCanvas();
}
document.querySelectorAll('#mapTools button').forEach((b) => b.addEventListener('click', () => setTool(b.dataset.tool)));

function updateRouteBar() {
  $('routeCount').textContent = `${routePoints.length} nöqtə`;
  $('routeStart').disabled = routePoints.length === 0;
}

$('routeClear').addEventListener('click', () => {
  routePoints = [];
  updateRouteBar();
  requestDraw();
});

$('routeStart').addEventListener('click', () => {
  if (!routePoints.length) return;
  const loops = Math.max(0, parseInt($('routeLoops').value || '0', 10));
  if (send('route', { points: routePoints.map((p) => [p.x, p.y, p.yaw]), loops })) {
    routePoints = [];
    updateRouteBar();
    setTool('view');
  }
});

async function place(x, y, yaw) {
  if (tool === 'goto') {
    if (await ask(`Robot (${fmt(x)}, ${fmt(y)}) nöqtəsinə getsin?`, { okLabel: 'Get' })) {
      send('goto', { x, y, yaw });
    }
  } else if (tool === 'route') {
    routePoints.push({ x, y, yaw });
    updateRouteBar();
  } else if (tool === 'table') {
    const name = await askText(`Masa adı (${fmt(x)}, ${fmt(y)}):`, 'məs. 5');
    if (name) {
      const yawDeg = yaw == null ? 0 : (yaw * 180) / Math.PI;
      send('table_set', { name, x, y, yaw_deg: yawDeg });
    }
  } else if (tool === 'home') {
    const heading = yaw != null ? yaw : 0;
    if (await ask(`Ev (şarj yeri) bura təyin edilsin? (${fmt(x)}, ${fmt(y)}, ${deg(heading)}°)`)) {
      send('home_set', { x, y, yaw: heading });
      setTool('view');
    }
  } else if (tool === 'pose') {
    const heading = yaw != null ? yaw : (state && state.pose ? state.pose.yaw : 0);
    if (await ask('Robotun mövqeyi bura təyin edilsin?')) send('set_pose', { x, y, yaw: heading });
  }
}

// ---------- gestures: pan, pinch zoom, tap/drag placing ----------

const pointers = new Map();
let gesture = null;

function pinchState() {
  const [a, b] = [...pointers.values()];
  const mx = (a.x + b.x) / 2;
  const my = (a.y + b.y) / 2;
  return { dist: Math.hypot(a.x - b.x, a.y - b.y) || 1, mx, my };
}

canvas.addEventListener('pointerdown', (e) => {
  canvas.setPointerCapture(e.pointerId);
  pointers.set(e.pointerId, { x: e.offsetX, y: e.offsetY });
  if (pointers.size === 2) {
    const p = pinchState();
    const [wx, wy] = s2w(p.mx, p.my);
    gesture = { type: 'pinch', dist0: p.dist, scale0: view.scale, wx, wy };
    dragPreview = null;
  } else if (pointers.size === 1) {
    gesture = {
      type: tool === 'view' ? 'pan' : 'place',
      x0: e.offsetX, y0: e.offsetY, cx0: view.cx, cy0: view.cy, moved: false,
    };
    if (gesture.type === 'place') {
      const [wx, wy] = s2w(e.offsetX, e.offsetY);
      dragPreview = { x: wx, y: wy, yaw: null };
      requestDraw();
    }
  }
});

canvas.addEventListener('pointermove', (e) => {
  if (!pointers.has(e.pointerId)) return;
  pointers.set(e.pointerId, { x: e.offsetX, y: e.offsetY });
  if (!gesture) return;
  if (gesture.type === 'pinch' && pointers.size === 2) {
    const p = pinchState();
    view.scale = Math.min(400, Math.max(4, gesture.scale0 * (p.dist / gesture.dist0)));
    // keep the world point that was under the fingers under the fingers
    view.cx = gesture.wx - (p.mx - cw / 2) / view.scale;
    view.cy = gesture.wy + (p.my - ch / 2) / view.scale;
    setFollow(false);
    requestDraw();
  } else if (gesture.type === 'pan') {
    const dx = e.offsetX - gesture.x0;
    const dy = e.offsetY - gesture.y0;
    if (Math.hypot(dx, dy) > 4) {
      gesture.moved = true;
      setFollow(false);
    }
    view.cx = gesture.cx0 - dx / view.scale;
    view.cy = gesture.cy0 + dy / view.scale;
    requestDraw();
  } else if (gesture.type === 'place' && dragPreview) {
    const dx = e.offsetX - gesture.x0;
    const dy = e.offsetY - gesture.y0;
    if (Math.hypot(dx, dy) > 14) {
      gesture.moved = true;
      dragPreview.yaw = Math.atan2(-dy, dx);
      requestDraw();
    }
  }
});

function endPointer(e) {
  if (!pointers.has(e.pointerId)) return;
  pointers.delete(e.pointerId);
  if (gesture && gesture.type === 'place' && pointers.size === 0 && dragPreview && e.type === 'pointerup') {
    const p = dragPreview;
    dragPreview = null;
    requestDraw();
    place(p.x, p.y, p.yaw);
  }
  if (pointers.size === 0) {
    gesture = null;
    dragPreview = null;
    requestDraw();
  }
}
canvas.addEventListener('pointerup', endPointer);
canvas.addEventListener('pointercancel', endPointer);

canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const [wx, wy] = s2w(e.offsetX, e.offsetY);
  view.scale = Math.min(400, Math.max(4, view.scale * (e.deltaY < 0 ? 1.15 : 1 / 1.15)));
  view.cx = wx - (e.offsetX - cw / 2) / view.scale;
  view.cy = wy + (e.offsetY - ch / 2) / view.scale;
  setFollow(false);
  requestDraw();
}, { passive: false });

function setFollow(on) {
  view.follow = on;
  $('followBtn').classList.toggle('on', on);
}
$('followBtn').addEventListener('click', () => {
  setFollow(!view.follow);
  if (view.follow && state && state.pose) {
    view.cx = state.pose.x;
    view.cy = state.pose.y;
  }
  requestDraw();
});

function setShowScan(on) {
  showScan = on;
  save('showScan', on);
  $('scanBtn').classList.toggle('on', on);
  $('legend').classList.toggle('hidden', !on);
  requestDraw();
}
$('scanBtn').addEventListener('click', () => setShowScan(!showScan));
setShowScan(showScan);

$('zoomIn').addEventListener('click', () => { view.scale = Math.min(400, view.scale * 1.3); requestDraw(); });
$('zoomOut').addEventListener('click', () => { view.scale = Math.max(4, view.scale / 1.3); requestDraw(); });

// ======================= camera =======================

let camKind = load('camKind', 'color');
let camRunning = false;

function camActive() {
  return camKind !== 'off' && $('tab-manual').classList.contains('active') && !document.hidden;
}

function camMessage(text) {
  $('camMsg').textContent = text;
  $('camMsg').classList.toggle('hidden', !text);
}

async function camFetch() {
  try {
    const r = await fetch(`/api/camera.jpg?kind=${camKind}&t=${Date.now()}`, { cache: 'no-store' });
    if (r.status !== 200) {
      camMessage(state && !state.sensors.camera ? 'Kamera qoşulmayıb' : 'Görüntü yüklənir…');
      return 500;
    }
    const blob = await r.blob();
    const img = $('camImg');
    const old = img.src;
    img.src = URL.createObjectURL(blob);
    if (old.startsWith('blob:')) URL.revokeObjectURL(old);
    camMessage('');
    return 0;
  } catch (e) {
    camMessage('Robotla əlaqə yoxdur');
    return 1000;
  }
}

async function camLoop() {
  if (camRunning || !camActive()) return;
  camRunning = true;
  while (camActive()) {
    const t0 = performance.now();
    const backoff = await camFetch();
    // ~8 pictures a second at most: plenty for driving, light on Wi-Fi.
    const wait = Math.max(backoff, 125 - (performance.now() - t0), 20);
    await new Promise((resolve) => setTimeout(resolve, wait));
  }
  camRunning = false;
}

function setCam(kind) {
  camKind = kind;
  save('camKind', kind);
  document.querySelectorAll('#camSeg button').forEach((b) => b.classList.toggle('on', b.dataset.cam === kind));
  $('depthScale').classList.toggle('hidden', kind !== 'depth');
  const img = $('camImg');
  if (img.src.startsWith('blob:')) URL.revokeObjectURL(img.src);
  img.removeAttribute('src');
  camMessage(kind === 'off' ? 'Kamera görüntüsü söndürülüb' : 'Görüntü yüklənir…');
  camLoop();
}
document.querySelectorAll('#camSeg button').forEach((b) => b.addEventListener('click', () => setCam(b.dataset.cam)));
document.addEventListener('visibilitychange', camLoop);

// ======================= joystick =======================

const joy = $('joy');
const knob = $('joyKnob');
let joyPointer = null;
let joyVec = { x: 0, y: 0 };
let joyTimer = null;

function updateJoy(e) {
  const r = joy.getBoundingClientRect();
  const radius = r.width / 2;
  let dx = e.clientX - (r.left + radius);
  let dy = e.clientY - (r.top + radius);
  const max = radius * 0.7;
  const d = Math.hypot(dx, dy);
  if (d > max) {
    dx *= max / d;
    dy *= max / d;
  }
  knob.style.transform = `translate(${dx}px, ${dy}px)`;
  joyVec = { x: dx / max, y: -dy / max };
}

function stopJoy() {
  joyPointer = null;
  joyVec = { x: 0, y: 0 };
  knob.style.transform = '';
  if (joyTimer) {
    clearInterval(joyTimer);
    joyTimer = null;
  }
  send('joy', { x: 0, y: 0 }, true);
}

joy.addEventListener('pointerdown', (e) => {
  if (joyPointer !== null) return;
  joy.setPointerCapture(e.pointerId);
  joyPointer = e.pointerId;
  updateJoy(e);
  send('joy', joyVec, true);
  // The robot stops by itself if these stop arriving (lost Wi-Fi, closed tab).
  joyTimer = setInterval(() => send('joy', joyVec, true), 100);
});
joy.addEventListener('pointermove', (e) => { if (e.pointerId === joyPointer) updateJoy(e); });
joy.addEventListener('pointerup', (e) => { if (e.pointerId === joyPointer) stopJoy(); });
joy.addEventListener('pointercancel', (e) => { if (e.pointerId === joyPointer) stopJoy(); });
document.addEventListener('visibilitychange', () => { if (document.hidden && joyPointer !== null) stopJoy(); });

// ======================= tables =======================

let tableSel = [];

function renderTables() {
  const tables = (state && state.tables) || {};
  tableSel = tableSel.filter((n) => n in tables);
  const names = Object.keys(tables).sort((a, b) => a.localeCompare(b, 'az', { numeric: true }));
  const list = $('tableList');
  list.innerHTML = '';
  if (!names.length) {
    list.innerHTML = '<p class="muted">Hələ masa yoxdur.</p>';
    return;
  }
  for (const name of names) {
    const t = tables[name];
    const row = document.createElement('div');
    row.className = 'item';
    const idx = tableSel.indexOf(name);

    const check = document.createElement('input');
    check.type = 'checkbox';
    check.checked = idx >= 0;
    check.addEventListener('change', () => {
      if (check.checked) tableSel.push(name);
      else tableSel = tableSel.filter((n) => n !== name);
      renderTables();
    });

    const order = document.createElement('span');
    order.className = 'order';
    order.textContent = idx >= 0 ? String(idx + 1) : '';
    if (idx < 0) order.style.visibility = 'hidden';

    const label = document.createElement('div');
    label.className = 'name';
    const b = document.createElement('b');
    b.textContent = name;
    const small = document.createElement('small');
    small.className = 'muted';
    small.textContent = `x ${fmt(t.x)}, y ${fmt(t.y)}, ${deg(t.yaw)}°`;
    label.append(b, small);

    const go = document.createElement('button');
    go.className = 'small primary';
    go.textContent = 'Get';
    go.addEventListener('click', () => send('goto_table', { name }));

    const del = document.createElement('button');
    del.className = 'small';
    del.textContent = 'Sil';
    del.addEventListener('click', async () => {
      if (await ask(`"${name}" masası silinsin?`, { okLabel: 'Sil', danger: true })) {
        send('table_delete', { name });
      }
    });

    row.append(check, order, label, go, del);
    list.appendChild(row);
  }
}

$('hereBtn').addEventListener('click', () => {
  const name = $('hereName').value.trim();
  if (!name) return toast('Masa adını yazın', 'warn');
  if (send('table_here', { name })) $('hereName').value = '';
});

$('tAdd').addEventListener('click', () => {
  const name = $('tName').value.trim();
  const x = parseFloat($('tX').value);
  const y = parseFloat($('tY').value);
  if (!name || Number.isNaN(x) || Number.isNaN(y)) return toast('Ad, x və y lazımdır', 'warn');
  const yawDeg = parseFloat($('tYaw').value) || 0;
  if (send('table_set', { name, x, y, yaw_deg: yawDeg })) {
    ['tName', 'tX', 'tY', 'tYaw'].forEach((id) => { $(id).value = ''; });
  }
});

$('tRoute').addEventListener('click', () => {
  if (!tableSel.length) return toast('Əvvəlcə masaları seçin', 'warn');
  const loops = Math.max(0, parseInt($('tLoops').value || '0', 10));
  send('route_tables', { names: tableSel, loops });
});

// ======================= maps / mapping =======================

function renderMaps() {
  const list = $('mapList');
  list.innerHTML = '';
  for (const name of state.maps) {
    const row = document.createElement('div');
    row.className = 'item';
    const label = document.createElement('div');
    label.className = 'name';
    label.textContent = name;
    row.appendChild(label);
    if (name === state.active_map) {
      const badge = document.createElement('span');
      badge.className = 'badge';
      badge.textContent = 'aktiv';
      row.appendChild(badge);
    } else {
      const use = document.createElement('button');
      use.className = 'small';
      use.textContent = 'Aktiv et';
      use.addEventListener('click', async () => {
        if (await ask(`"${name}" aktiv edilsin? Naviqasiya bu xəritə ilə yenidən başlayacaq.`)) {
          send('activate_map', { name });
        }
      });
      row.appendChild(use);
    }
    list.appendChild(row);
  }
}

$('autoMapBtn').addEventListener('click', async () => {
  const ok = await ask('Avtonom xəritələmə başlasın? Naviqasiya dayanacaq və robot özü hərəkət edəcək. ' +
    'Ətrafda insanlara diqqət edin; lazım olsa STOP basın.', { okLabel: 'Başlat' });
  if (ok && send('mode', { mode: 'mapping', explore: true })) showTab('map');
});
$('manualMapBtn').addEventListener('click', async () => {
  const ok = await ask('Manual xəritələmə başlasın? Naviqasiya dayanacaq. Robotu joystik ilə yavaş-yavaş ' +
    'bütün ərazidə gəzdirin, sonra xəritəni saxlayın.', { okLabel: 'Başlat' });
  if (ok && send('mode', { mode: 'mapping' })) showTab('manual');
});
$('startNav').addEventListener('click', async () => {
  if (await ask('Saxlanmamış xəritə itəcək. Naviqasiyaya qayıdılsın?', { okLabel: 'Qayıt', danger: true })) {
    send('mode', { mode: 'navigation' });
  }
});
$('exploreOn').addEventListener('click', () => send('explore', { on: true }));
$('exploreOff').addEventListener('click', () => send('explore', { on: false }));
$('saveMap').addEventListener('click', () => {
  const name = $('mapName').value.trim();
  if (!/^[A-Za-z0-9_-]{1,40}$/.test(name)) return toast('Ad: yalnız latın hərfləri, rəqəm, _ və -', 'warn');
  send('save_map', { name, activate: $('mapActivate').checked });
});

// ======================= home / battery =======================

async function goHome() {
  if (!state) return;
  if (!state.home) {
    toast('Ev nöqtəsi təyin edilməyib: Ayarlar → Ev və şarj', 'warn');
    return;
  }
  if (await ask('Robot evə (şarj yerinə) qayıtsın?', { okLabel: 'Evə qayıt' })) send('home_go');
}
$('homeBtn').addEventListener('click', goHome);
$('homeGo').addEventListener('click', goHome);
$('homeHere').addEventListener('click', async () => {
  if (await ask('Robotun indiki yeri və istiqaməti ev (şarj yeri) kimi saxlanılsın?', { okLabel: 'Saxla' })) {
    send('home_set_here');
  }
});
$('autoHome').addEventListener('change', () => {
  sliderBusy.autoHome = true;
  send('auto_home', { on: $('autoHome').checked });
  setTimeout(() => { sliderBusy.autoHome = false; }, 800);
});

// ======================= misc buttons =======================

$('stopBtn').addEventListener('click', () => send('stop'));
$('navCancel').addEventListener('click', () => send('stop'));
$('relocBtn').addEventListener('click', async () => {
  if (await ask('Robot əvvəlcə yerində durub lidarla axtaracaq; tapılmasa yerində iki dəfə dönəcək. '
                + 'Ətraf boşdur?', { okLabel: 'Başlat' })) send('relocalize');
});
$('restartBtn').addEventListener('click', async () => {
  if (await ask('Naviqasiya yenidən başladılsın?', { okLabel: 'Yenidən başlat' })) send('restart');
});

// ======================= object recognition =======================

const OBJECT_NAMES = { person: 'insan', chair: 'stul', table: 'masa', sofa: 'divan' };
const OBJECT_COLORS = { person: '#f97316', chair: '#3b82f6', table: '#16a34a', sofa: '#a855f7' };
const PERCEPTION_STATUS = {
  ok: 'işləyir (GPU)', 'ok (CPU)': 'işləyir (CPU, yavaş)', loading: 'model yüklənir…',
  'no camera': 'kamera görüntüsü yoxdur', '': 'tanıma node-u işləmir',
};
let lastCandJson = '';

function renderPerception(s) {
  const p = s.perception;
  $('percText').textContent = PERCEPTION_STATUS[p.status] ||
    (p.status.startsWith('error') ? `xəta: ${p.status.slice(7)}` : p.status);
  if (!sliderBusy.peopleGuard) $('peopleGuard').checked = p.people_guard;
  if (!sliderBusy.tableApproach) $('tableApproach').checked = p.table_approach;
  syncSlider('tableGap', 'tableGapVal', Math.round(p.table_gap * 100), [5, 60], (v) => `${v} sm`);

  const bar = $('peopleBar');
  bar.classList.toggle('hidden', !s.people);
  bar.textContent = s.people === 'wait' ? 'Qarşıda insan var — keçməsini gözləyir'
    : 'İnsanlar yaxındır — yavaş gedir';

  const candJson = JSON.stringify(s.candidates);
  if (candJson !== lastCandJson) {
    lastCandJson = candJson;
    renderCandidates(s.candidates);
  }
  drawCamOverlay();
}

function renderCandidates(cands) {
  $('candCard').classList.toggle('hidden', !cands.length);
  const list = $('candList');
  list.innerHTML = '';
  for (const c of cands) {
    const row = document.createElement('div');
    row.className = 'item';
    const label = document.createElement('div');
    label.className = 'name';
    const b = document.createElement('b');
    b.textContent = `Masa təklifi #${c.id}`;
    const small = document.createElement('small');
    small.className = 'muted';
    small.textContent = `x ${fmt(c.x)}, y ${fmt(c.y)} · ${c.n} dəfə görülüb`;
    label.append(b, small);
    const keep = document.createElement('button');
    keep.className = 'small primary';
    keep.textContent = 'Saxla';
    keep.addEventListener('click', async () => {
      const name = await askText(`Masa təklifi #${c.id} üçün ad:`, 'məs. 5');
      if (name) send('candidate_accept', { id: c.id, name });
    });
    const drop = document.createElement('button');
    drop.className = 'small';
    drop.textContent = 'Rədd et';
    drop.addEventListener('click', async () => {
      if (await ask(`Təklif #${c.id} rədd edilsin? Bu yerdə bir daha təklif olunmayacaq.`,
        { okLabel: 'Rədd et', danger: true })) send('candidate_dismiss', { id: c.id });
    });
    row.append(label, keep, drop);
    list.appendChild(row);
  }
}

function drawCamOverlay() {
  const cv = $('camOverlay');
  const r = cv.getBoundingClientRect();
  if (!r.width) return;
  const dpr = window.devicePixelRatio || 1;
  if (cv.width !== Math.round(r.width * dpr) || cv.height !== Math.round(r.height * dpr)) {
    cv.width = Math.round(r.width * dpr);
    cv.height = Math.round(r.height * dpr);
  }
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, r.width, r.height);
  const img = $('camImg');
  // Boxes are in colour-image coordinates; the depth picture has another field of view.
  if (camKind !== 'color' || !state || !state.objects || !img.naturalWidth || !img.getAttribute('src')) return;
  const scale = Math.min(r.width / img.naturalWidth, r.height / img.naturalHeight);
  const dw = img.naturalWidth * scale;
  const dh = img.naturalHeight * scale;
  const ox = (r.width - dw) / 2;
  const oy = (r.height - dh) / 2;
  g.font = 'bold 12px system-ui';
  g.lineWidth = 2;
  for (const o of state.objects) {
    const [x1, y1, x2, y2] = o.box;
    const color = OBJECT_COLORS[o.cls] || '#fff';
    const x = ox + x1 * dw;
    const y = oy + y1 * dh;
    g.strokeStyle = color;
    g.strokeRect(x, y, (x2 - x1) * dw, (y2 - y1) * dh);
    const text = `${OBJECT_NAMES[o.cls] || o.cls}${o.dist != null ? ` ${o.dist.toFixed(1)} m` : ''}`;
    const tw = g.measureText(text).width + 8;
    g.fillStyle = color;
    g.fillRect(x, Math.max(0, y - 17), tw, 17);
    g.fillStyle = '#fff';
    g.fillText(text, x + 4, Math.max(12, y - 5));
  }
}
$('camImg').addEventListener('load', drawCamOverlay);

function drawObjects() {
  if (!state) return;
  if (showScan) {
    for (const o of state.objects || []) {
      if (!o.map) continue;
      const [sx, sy] = w2s(o.map[0], o.map[1]);
      const color = OBJECT_COLORS[o.cls] || '#fff';
      ctx.fillStyle = color;
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      if (o.cls === 'person') {
        ctx.beginPath();
        ctx.arc(sx, sy, Math.max(6, 0.25 * view.scale), 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      } else {
        const half = Math.max(5, (o.cls === 'chair' ? 0.2 : 0.3) * view.scale);
        ctx.fillRect(sx - half, sy - half, 2 * half, 2 * half);
        ctx.strokeRect(sx - half, sy - half, 2 * half, 2 * half);
      }
    }
  }
  for (const c of state.candidates || []) {
    const [sx, sy] = w2s(c.x, c.y);
    ctx.setLineDash([5, 4]);
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(sx, sy, Math.max(8, c.r * view.scale), 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.font = 'bold 14px system-ui';
    ctx.fillStyle = '#b45309';
    ctx.textAlign = 'center';
    ctx.fillText('?', sx, sy + 5);
    ctx.textAlign = 'left';
  }
}

$('peopleGuard').addEventListener('change', () => {
  sliderBusy.peopleGuard = true;
  send('perception', { people_guard: $('peopleGuard').checked });
  setTimeout(() => { sliderBusy.peopleGuard = false; }, 800);
});
$('tableApproach').addEventListener('change', () => {
  sliderBusy.tableApproach = true;
  send('perception', { table_approach: $('tableApproach').checked });
  setTimeout(() => { sliderBusy.tableApproach = false; }, 800);
});
bindSlider('tableGap', 'tableGapVal', (v) => send('perception', { table_gap: v / 100 }), (v) => `${v} sm`);

// ======================= robot profile editor =======================
// Dimensions and sensor positions; the server validates them, derives the
// footprint etc., and a saved profile takes effect after a restart.

const DRIVE_LABELS = { skid4: '4 aparıcı təkər (tank dönmə)', diff2: '2 aparıcı təkər + sərbəst təkərlər' };
// [section, key, label, unit, scale (display = value * scale), step, visible(profile)]
const ROBOT_GROUPS = [
  ['Platforma', [
    ['platform', 'length', 'Uzunluq', 'sm', 100, 0.5],
    ['platform', 'width', 'En', 'sm', 100, 0.5],
    ['platform', 'height', 'Hündürlük', 'sm', 100, 0.5],
  ]],
  ['Hərəkət sistemi', [
    ['drive', 'type', 'Növ', '', 0, 0],
    ['drive', 'wheel_diameter', 'Təkər diametri', 'mm', 1000, 0.5],
    ['drive', 'wheel_width', 'Təkər eni', 'mm', 1000, 1],
    ['drive', 'track', 'Sol–sağ təkər mərkəzləri arası', 'sm', 100, 0.1],
    ['drive', 'wheelbase', 'Ön–arxa ox arası', 'sm', 100, 0.1, (p) => p.drive.type === 'skid4'],
    ['drive', 'center_x', 'Təkərlərin mərkəzi, platformanın mərkəzindən irəli +', 'sm', 100, 0.5],
    ['drive', 'ticks_per_rev_left', 'Sol enkoder, impuls / təkər dövrü', '', 1, 1],
    ['drive', 'ticks_per_rev_right', 'Sağ enkoder, impuls / təkər dövrü', '', 1, 1],
    ['drive', 'turn_slip_factor', 'Dönmə sürüşmə əmsalı, 1 = sürüşmə yoxdur', '', 1, 0.05],
    ['drive', 'max_speed', 'Maksimal sürət (manual və avtonom)', 'km/saat', 3.6, 0.1],
    ['drive', 'reversed', 'Ötürücü tərs quraşdırılıb (mühərriklərin irəlisi kameranın əksinədir)', '', 0, 0],
  ]],
  ['Lidar', [
    ['lidar', 'x', 'x, irəli +', 'sm', 100, 0.5],
    ['lidar', 'y', 'y, sola +', 'sm', 100, 0.5],
    ['lidar', 'height', 'Yerdən hündürlük', 'sm', 100, 0.5],
    ['lidar', 'yaw', 'Bucaq, 0 = irəli', '°', 1, 1],
    ['lidar', 'mask', 'Robotun öz hissəsini gizlət', '', 0, 0],
    ['lidar', 'mask_from', 'Gizlədilən sektor: başlanğıc', '°', 1, 1, (p) => p.lidar.mask],
    ['lidar', 'mask_to', 'Gizlədilən sektor: son', '°', 1, 1, (p) => p.lidar.mask],
  ]],
  ['Kamera', [
    ['camera', 'x', 'x, irəli +', 'sm', 100, 0.5],
    ['camera', 'y', 'y, sola +', 'sm', 100, 0.5],
    ['camera', 'height', 'Yerdən hündürlük', 'sm', 100, 0.5],
    ['camera', 'yaw', 'Bucaq, 0 = irəli', '°', 1, 1],
  ]],
  ['Təhlükəsizlik', [
    ['safety', 'padding', 'Kontur ehtiyatı', 'sm', 100, 0.5],
    ['safety', 'extra_inflation', 'Maneədən əlavə uzaq durma', 'sm', 100, 1],
  ]],
];

let robotSaved = null;   // {profile, derived} as last loaded from the server
let robotDraft = null;   // profile being edited
let robotDerived = null;
let robotPreviewTimer = null;

const clone = (o) => JSON.parse(JSON.stringify(o));
const cm = (m) => Math.round(m * 1000) / 10;

function robotSummary(info) {
  const p = info.profile;
  const d = info.derived;
  return `${DRIVE_LABELS[p.drive.type]} · təkər ⌀${Math.round(p.drive.wheel_diameter * 10000) / 10} mm · ` +
    `iz ${cm(p.drive.track)} sm · kontur ${cm(d.size.length)} × ${cm(d.size.width)} sm`;
}

function buildRobotForm() {
  const form = $('robotForm');
  form.innerHTML = '';
  for (const [title, fields] of ROBOT_GROUPS) {
    const group = document.createElement('div');
    group.className = 'fgroup';
    const h = document.createElement('h4');
    h.textContent = title;
    const grid = document.createElement('div');
    grid.className = 'fgrid';
    for (const [section, key, label, unit, scale, step, visible] of fields) {
      const field = document.createElement('label');
      field.className = 'field';
      field.dataset.section = section;
      field.dataset.key = key;
      const text = document.createElement('span');
      text.textContent = unit ? `${label} (${unit})` : label;
      let input;
      if (key === 'type') {
        input = document.createElement('select');
        for (const [value, name] of Object.entries(DRIVE_LABELS)) {
          const o = document.createElement('option');
          o.value = value;
          o.textContent = name;
          input.appendChild(o);
        }
        field.classList.add('wide');
      } else if (key === 'mask' || key === 'reversed') {
        input = document.createElement('input');
        input.type = 'checkbox';
        field.classList.add('wide', 'inline');
      } else {
        input = document.createElement('input');
        input.type = 'number';
        input.inputMode = 'decimal';
        input.step = String(step);
        if (label.length > 26) field.classList.add('wide');
      }
      input.addEventListener(input.type === 'checkbox' || key === 'type' ? 'change' : 'input', () => {
        readRobotForm();
        updateRobotVisibility();
        scheduleRobotPreview();
      });
      field._meta = { section, key, scale, visible };
      field.append(text, input);
      grid.appendChild(field);
    }
    group.append(h, grid);
    form.appendChild(group);
  }
}

function robotFields() {
  return [...$('robotForm').querySelectorAll('.field')];
}

function fillRobotForm(profile) {
  for (const field of robotFields()) {
    const { section, key, scale } = field._meta;
    const input = field.querySelector('input, select');
    const value = profile[section][key];
    if (input.type === 'checkbox') input.checked = !!value;
    else if (input.tagName === 'SELECT') input.value = value;
    else input.value = String(Math.round(value * scale * 100) / 100);
  }
  updateRobotVisibility();
}

function readRobotForm() {
  for (const field of robotFields()) {
    const { section, key, scale } = field._meta;
    const input = field.querySelector('input, select');
    if (input.type === 'checkbox') robotDraft[section][key] = input.checked;
    else if (input.tagName === 'SELECT') robotDraft[section][key] = input.value;
    else if (input.value !== '') robotDraft[section][key] = Number(input.value) / scale;
  }
}

function updateRobotVisibility() {
  for (const field of robotFields()) {
    const { visible } = field._meta;
    field.classList.toggle('hidden', !!visible && !visible(robotDraft));
  }
}

async function robotRequest(body) {
  const init = body ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {};
  const r = await fetch('/api/robot', { cache: 'no-store', ...init });
  if (!r.ok) throw new Error(String(r.status));
  return r.json();
}

async function loadRobot() {
  try {
    robotSaved = await robotRequest();
  } catch (e) {
    return;
  }
  $('robotSummary').textContent = robotSummary(robotSaved);
  if ($('robotEditor').classList.contains('hidden')) {
    robotDraft = clone(robotSaved.profile);
    robotDerived = robotSaved.derived;
  }
}

function showRobotResult(info) {
  robotDerived = info.derived;
  const d = info.derived;
  $('robotDerived').textContent =
    `Kontur: ${cm(d.size.length)} × ${cm(d.size.width)} sm · Dönəndə tutduğu dairə: ⌀${cm(2 * d.circumscribed_radius)} sm · ` +
    `Maneədən uzaq durma: ${cm(d.inflation_radius)} sm`;
  const errors = $('robotErrors');
  errors.classList.toggle('hidden', !info.errors.length);
  errors.textContent = info.errors.length ? `Yanlış dəyərlər: ${info.errors.join(', ')}` : '';
  $('robotSave').disabled = info.errors.length > 0;
  drawRobotPreview();
}

function scheduleRobotPreview() {
  clearTimeout(robotPreviewTimer);
  robotPreviewTimer = setTimeout(async () => {
    try {
      showRobotResult(await robotRequest({ profile: robotDraft, save: false }));
    } catch (e) {
      toast('Robotla əlaqə yoxdur', 'error');
    }
  }, 250);
}

function drawRobotPreview() {
  const c = $('robotPreview');
  const r = c.getBoundingClientRect();
  if (!r.width || !robotDerived) return;
  const dpr = window.devicePixelRatio || 1;
  c.width = Math.round(r.width * dpr);
  c.height = Math.round(r.height * dpr);
  const g = c.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, r.width, r.height);

  const p = robotDraft;
  const d = robotDerived;
  const cx = p.drive.center_x;
  // Everything in rotation-centre coordinates; forward is up on screen.
  const pts = [...d.footprint, [d.lidar.x, d.lidar.y], [d.camera.x, d.camera.y]];
  const ext = Math.max(...pts.map(([x, y]) => Math.max(Math.abs(x), Math.abs(y)))) + 0.12;
  const k = Math.min(r.width, r.height) / (2 * ext);
  const ox = r.width / 2;
  const oy = r.height / 2;
  const S = (x, y) => [ox - y * k, oy - x * k];

  g.strokeStyle = 'rgba(139,154,179,.15)';
  g.lineWidth = 1;
  for (let v = -Math.ceil(ext * 10) / 10; v <= ext; v += 0.1) {
    const [a] = S(0, v);
    const [, b] = S(v, 0);
    g.beginPath(); g.moveTo(a, 0); g.lineTo(a, r.height); g.stroke();
    g.beginPath(); g.moveTo(0, b); g.lineTo(r.width, b); g.stroke();
  }

  // platform
  const pl = p.platform;
  const [px, py] = S(pl.length / 2 - cx, pl.width / 2);
  g.fillStyle = '#475569';
  g.fillRect(px, py, pl.width * k, pl.length * k);

  // wheels
  g.fillStyle = '#0f172a';
  g.strokeStyle = '#94a3b8';
  for (const w of d.wheels) {
    const [wx, wy] = S(w.x + d.wheel_radius, w.y + p.drive.wheel_width / 2);
    g.fillRect(wx, wy, p.drive.wheel_width * k, 2 * d.wheel_radius * k);
    g.strokeRect(wx, wy, p.drive.wheel_width * k, 2 * d.wheel_radius * k);
  }

  // footprint
  g.setLineDash([6, 4]);
  g.strokeStyle = '#ef4444';
  g.lineWidth = 2;
  g.beginPath();
  d.footprint.forEach(([x, y], i) => { const [a, b] = S(x, y); if (i) g.lineTo(a, b); else g.moveTo(a, b); });
  g.closePath();
  g.stroke();
  g.setLineDash([]);

  const deg2rad = Math.PI / 180;
  const wedge = (x, y, yawDeg, fromDeg, toDeg, len, fill) => {
    const [a, b] = S(x, y);
    g.fillStyle = fill;
    g.beginPath();
    g.moveTo(a, b);
    for (let t = fromDeg; t <= toDeg; t += 2) {
      const th = (yawDeg + t) * deg2rad;
      const [ex, ey] = S(x + Math.cos(th) * len, y + Math.sin(th) * len);
      g.lineTo(ex, ey);
    }
    g.closePath();
    g.fill();
  };

  // camera: D435 depth field of view (~87 deg)
  wedge(d.camera.x, d.camera.y, p.camera.yaw, -43.5, 43.5, 0.35, 'rgba(34,197,94,.18)');
  {
    const [a, b] = S(d.camera.x, d.camera.y);
    g.save();
    g.translate(a, b);
    g.rotate(-p.camera.yaw * deg2rad);
    g.fillStyle = '#22c55e';
    g.fillRect(-0.045 * k, -0.0125 * k, 0.09 * k, 0.025 * k);
    g.restore();
  }

  // lidar, with the hidden sector
  if (p.lidar.mask) {
    const to = p.lidar.mask_to >= p.lidar.mask_from ? p.lidar.mask_to : p.lidar.mask_to + 360;
    wedge(d.lidar.x, d.lidar.y, 0, p.lidar.mask_from, to, 0.3, 'rgba(239,68,68,.35)');
  }
  {
    const [a, b] = S(d.lidar.x, d.lidar.y);
    g.fillStyle = '#3b82f6';
    g.beginPath(); g.arc(a, b, Math.max(0.035 * k, 4), 0, Math.PI * 2); g.fill();
    const th = p.lidar.yaw * deg2rad;
    const [ex, ey] = S(d.lidar.x + Math.cos(th) * 0.08, d.lidar.y + Math.sin(th) * 0.08);
    g.strokeStyle = '#fff'; g.lineWidth = 2;
    g.beginPath(); g.moveTo(a, b); g.lineTo(ex, ey); g.stroke();
  }

  // rotation centre
  const [rx, ry] = S(0, 0);
  g.strokeStyle = '#f59e0b'; g.lineWidth = 2;
  g.beginPath(); g.moveTo(rx - 7, ry); g.lineTo(rx + 7, ry); g.moveTo(rx, ry - 7); g.lineTo(rx, ry + 7); g.stroke();

  g.fillStyle = '#e5ecf6';
  g.font = '12px system-ui';
  g.textAlign = 'center';
  g.fillText('▲ ÖN', ox, 14);
}

function openRobotEditor(open) {
  $('robotEditor').classList.toggle('hidden', !open);
  $('robotEditBtn').textContent = open ? 'Bağla' : 'Redaktə et';
  if (open && robotSaved) {
    robotDraft = clone(robotSaved.profile);
    fillRobotForm(robotDraft);
    showRobotResult(robotSaved);
  }
}

$('robotEditBtn').addEventListener('click', () => openRobotEditor($('robotEditor').classList.contains('hidden')));
$('robotReset').addEventListener('click', () => openRobotEditor(true));
$('robotSave').addEventListener('click', async () => {
  readRobotForm();
  const ok = await ask('Robot quruluşu yadda saxlanılsın və tətbiq olunsun? Robot təxminən 20 saniyəlik ' +
    'yenidən başlayacaq; hərəkətdədirsə dayanacaq.', { okLabel: 'Tətbiq et' });
  if (!ok) return;
  try {
    const info = await robotRequest({ profile: robotDraft, save: true, apply: true });
    if (info.errors.length) {
      showRobotResult(info);
      return;
    }
    robotSaved = { ...info };
    $('robotSummary').textContent = robotSummary(info);
    openRobotEditor(false);
  } catch (e) {
    toast('Yadda saxlamaq alınmadı', 'error');
  }
});
window.addEventListener('resize', drawRobotPreview);
buildRobotForm();
loadRobot();

setTool('view');
updateRouteBar();
setCam(camKind);
const startTab = location.hash.slice(1);
if (document.getElementById(`tab-${startTab}`)) showTab(startTab);
connect();
