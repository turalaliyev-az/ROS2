"""Keeps detections apart over time: stable ids, smoothed positions, speed (no ROS)."""
import itertools
import math


class Tracker:
    """
    Greedy nearest-neighbour tracking per class.

    Works in a frame fixed to the world (odom), so the robot's own motion
    doesn't look like the objects moving.
    """

    def __init__(self, match_radius=0.6, max_age=1.0, smoothing=0.5):
        self.match_radius = match_radius
        self.max_age = max_age
        self.smoothing = smoothing
        self.tracks = {}
        self._ids = itertools.count(1)

    def update(self, detections, now):
        """
        Match detections to tracks; return the live tracks.

        detections: dicts with 'cls' and 'pos' (x, y); each gets an 'id'.
        """
        free = {tid for tid, t in self.tracks.items() if now - t['seen'] <= self.max_age}
        pairs = []
        for i, det in enumerate(detections):
            for tid in free:
                t = self.tracks[tid]
                if t['cls'] != det['cls']:
                    continue
                d = math.hypot(t['pos'][0] - det['pos'][0], t['pos'][1] - det['pos'][1])
                if d <= self.match_radius:
                    pairs.append((d, i, tid))
        used_det, used_track = set(), set()
        for d, i, tid in sorted(pairs):
            if i in used_det or tid in used_track:
                continue
            used_det.add(i)
            used_track.add(tid)
            t = self.tracks[tid]
            det = detections[i]
            dt = max(now - t['seen'], 1e-3)
            a = self.smoothing
            new = (a * t['pos'][0] + (1 - a) * det['pos'][0],
                   a * t['pos'][1] + (1 - a) * det['pos'][1])
            speed = math.hypot(new[0] - t['pos'][0], new[1] - t['pos'][1]) / dt
            t.update(pos=new, seen=now, hits=t['hits'] + 1, speed=0.5 * t['speed'] + 0.5 * speed,
                     data=det)
            det['id'] = tid
        for i, det in enumerate(detections):
            if i not in used_det:
                tid = next(self._ids)
                self.tracks[tid] = {'cls': det['cls'], 'pos': tuple(det['pos']), 'seen': now,
                                    'hits': 1, 'speed': 0.0, 'data': det}
                det['id'] = tid
        for tid in [tid for tid, t in self.tracks.items() if now - t['seen'] > self.max_age]:
            del self.tracks[tid]
        return self.tracks
