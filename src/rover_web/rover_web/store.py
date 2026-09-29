"""Persistent robot data under ~/.rover: maps, tables and home per map, settings, last pose."""
import math
import os
import re
import secrets
import shutil
from pathlib import Path

import yaml

# ROVER_HOME lets a test instance run without touching the robot's real data.
ROOT = Path(os.environ.get('ROVER_HOME', Path.home() / '.rover'))
NAME_RE = re.compile(r'^[A-Za-z0-9_\-]{1,40}$')

DEFAULT_SETTINGS = {
    'active_map': None,
    'nav_speed': 0.2,
    'manual_speed': 0.15,
    'access_key': None,
    'auto_home': True,          # drive home when the PC battery runs low
    'auto_home_level': 20,      # percent
}


def _load(path, default):
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
        return data if data is not None else default
    except FileNotFoundError:
        return default


def _save(path, data):
    tmp = path.with_suffix(path.suffix + '.tmp')
    with open(tmp, 'w') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=True)
    tmp.replace(path)  # atomic: a power cut never leaves a half-written file


def valid_name(name):
    return bool(NAME_RE.match(name or ''))


class Store:

    def __init__(self, seed_map_yaml=None):
        self.maps_dir = ROOT / 'maps'
        self.tables_dir = ROOT / 'tables'
        self.logs_dir = ROOT / 'logs'
        for d in (self.maps_dir, self.tables_dir, self.logs_dir):
            d.mkdir(parents=True, exist_ok=True)

        self._tables = {}
        self._homes = _load(ROOT / 'home.yaml', {})
        self.settings_path = ROOT / 'settings.yaml'
        self.settings = dict(DEFAULT_SETTINGS)
        self.settings.update(_load(self.settings_path, {}))
        if not self.settings['access_key']:
            self.settings['access_key'] = secrets.token_urlsafe(9)

        if not self.list_maps() and seed_map_yaml and Path(seed_map_yaml).exists():
            self._import_map(Path(seed_map_yaml))
        if self.settings['active_map'] not in self.list_maps():
            maps = self.list_maps()
            self.settings['active_map'] = maps[0] if maps else None
        self.save_settings()

    # ---------------- settings ----------------

    def save_settings(self):
        _save(self.settings_path, self.settings)

    def set(self, key, value):
        self.settings[key] = value
        self.save_settings()

    # ---------------- maps ----------------

    def _import_map(self, yaml_path):
        meta = _load(yaml_path, {})
        image = yaml_path.parent / meta['image']
        name = yaml_path.stem
        shutil.copy(image, self.maps_dir / image.name)
        shutil.copy(yaml_path, self.maps_dir / yaml_path.name)
        self.settings['active_map'] = name

    def list_maps(self):
        return sorted(p.stem for p in self.maps_dir.glob('*.yaml'))

    def map_yaml(self, name):
        return str(self.maps_dir / f'{name}.yaml')

    def map_prefix(self, name):
        return str(self.maps_dir / name)

    # ---------------- tables ----------------

    def tables(self, map_name):
        # Read on every state broadcast (5 Hz), so keep a copy in memory.
        if not map_name:
            return {}
        if map_name not in self._tables:
            self._tables[map_name] = _load(self.tables_dir / f'{map_name}.yaml', {})
        return dict(self._tables[map_name])

    def _write_tables(self, map_name, tables):
        _save(self.tables_dir / f'{map_name}.yaml', tables)
        self._tables[map_name] = tables

    def set_table(self, map_name, name, x, y, yaw):
        tables = self.tables(map_name)
        tables[name] = {'x': round(float(x), 3), 'y': round(float(y), 3),
                        'yaw': round(float(yaw), 3)}
        self._write_tables(map_name, tables)
        return tables

    def delete_table(self, map_name, name):
        tables = self.tables(map_name)
        tables.pop(name, None)
        self._write_tables(map_name, tables)
        return tables

    # ---------------- home (charging spot) ----------------

    def home(self, map_name):
        pose = self._homes.get(map_name)
        if pose and all(k in pose for k in ('x', 'y', 'yaw')):
            return dict(pose)
        return None

    def set_home(self, map_name, x, y, yaw):
        self._homes[map_name] = {'x': round(float(x), 3), 'y': round(float(y), 3),
                                 'yaw': round(float(yaw), 4)}
        _save(ROOT / 'home.yaml', self._homes)

    # ---------------- last pose ----------------

    def last_pose(self, map_name):
        data = _load(ROOT / 'last_pose.yaml', {})
        pose = data.get(map_name)
        if pose and all(k in pose for k in ('x', 'y', 'yaw')):
            return pose
        return None

    def save_last_pose(self, map_name, x, y, yaw):
        if not map_name or not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        data = _load(ROOT / 'last_pose.yaml', {})
        data[map_name] = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4)}
        _save(ROOT / 'last_pose.yaml', data)
