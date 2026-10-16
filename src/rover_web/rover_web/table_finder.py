"""
Table candidates: tables the camera keeps seeing, offered to the operator to name.

Each sighting is the near side of a table in map coordinates (the camera can't
see past it); the middle is taken to be half the table's width further on,
along the line of sight. Sightings within MATCH_RADIUS of a candidate refine it,
at most one per MIN_SIGHTING_INTERVAL (the detector runs at ~8 Hz, so a brief
false detection would otherwise count many times); a candidate is offered once
it has been seen MIN_SIGHTINGS times. Its stopping pose is on the side the
robot saw it from (known to be reachable), facing it. Times are wall-clock
seconds, as candidates are saved across restarts.
"""
import math

MATCH_RADIUS = 0.7     # m
MIN_SIGHTINGS = 5
MIN_SIGHTING_INTERVAL = 1.0  # s: so an offer means a table seen for 4 s or more
STALE_AGE = 900.0      # s: a candidate never confirmed and not seen since is forgotten
MAX_WEIGHT = 50        # later sightings still move an old candidate a little
KNOWN_RADIUS = 1.2     # a named table's stopping point this close: already known
# The size estimate is rough (a low camera, a box around legs and edges), and a
# rectangular table seen corner-first reaches further than its width suggests:
# the Nav2 goal stays at least this far from the centre plus the robot, and the
# camera-guided approach closes the remaining gap.
MIN_STANDOFF = 0.8     # m


def table_center(near, robot, width):
    """(centre, radius) of a table whose near side is at `near`, seen from `robot`."""
    dx, dy = near[0] - robot[0], near[1] - robot[1]
    d = math.hypot(dx, dy) or 1.0
    radius = min(max(width / 2.0, 0.3), 0.8)
    return (near[0] + dx / d * radius, near[1] + dy / d * radius), radius


def add_sighting(candidates, center, radius, robot, next_id, now):
    """Merge one sighting into `candidates` (list, changed in place); returns the candidate."""
    vx, vy = robot[0] - center[0], robot[1] - center[1]
    norm = math.hypot(vx, vy) or 1.0
    vx, vy = vx / norm, vy / norm
    best = min(candidates, default=None,
               key=lambda c: math.hypot(c['x'] - center[0], c['y'] - center[1]))
    if best is not None and (math.hypot(best['x'] - center[0], best['y'] - center[1])
                             < MATCH_RADIUS):
        if now - best.get('last', 0.0) < MIN_SIGHTING_INTERVAL:
            return best
        w = 1.0 / (min(best['n'], MAX_WEIGHT) + 1)
        best['x'] += (center[0] - best['x']) * w
        best['y'] += (center[1] - best['y']) * w
        best['r'] += (radius - best['r']) * w
        best['vx'] += (vx - best['vx']) * w
        best['vy'] += (vy - best['vy']) * w
        best['n'] += 1
        best['last'] = now
        return best
    cand = {'id': next_id, 'x': center[0], 'y': center[1], 'r': radius,
            'vx': vx, 'vy': vy, 'n': 1, 'dismissed': False, 'last': now}
    candidates.append(cand)
    return cand


def prune(candidates, now):
    """Forget candidates never confirmed and not seen for STALE_AGE (in place); dismissed stay."""
    candidates[:] = [c for c in candidates
                     if c['dismissed'] or c['n'] >= MIN_SIGHTINGS
                     or now - c.get('last', 0.0) <= STALE_AGE]


def is_known(cand, tables):
    """Tell whether a named table already covers this candidate."""
    for t in tables.values():
        if 'center' in t:
            if math.hypot(t['center'][0] - cand['x'], t['center'][1] - cand['y']) < MATCH_RADIUS:
                return True
        elif math.hypot(t['x'] - cand['x'], t['y'] - cand['y']) < KNOWN_RADIUS:
            return True
    return False


def offered(candidates, tables):
    """Get the candidates to offer the operator for naming."""
    return [c for c in candidates
            if not c['dismissed'] and c['n'] >= MIN_SIGHTINGS and not is_known(c, tables)]


def approach_pose(cand, robot_front, gap=0.3):
    """Where the robot should stop for this table: (x, y, yaw facing the table)."""
    vx, vy = cand['vx'], cand['vy']
    norm = math.hypot(vx, vy) or 1.0
    vx, vy = vx / norm, vy / norm
    d = max(cand['r'], MIN_STANDOFF) + robot_front + gap
    return cand['x'] + vx * d, cand['y'] + vy * d, math.atan2(-vy, -vx)
