"""Session recorder — every telemetry packet and every estimate, as JSON lines.

A recording is what turns a walk around the room into data for the thesis:
tools/evaluate.py reads it back, compares the track with the path that was
walked (a taped line, a measured point) and can *replay* the raw packets
through the current tracker, so a tuning change is judged on the very same
measurements instead of on a new walk.

Line kinds (field "k"):
    meta   first line: name, note, scene (anchors, room, tag height, calibration)
    rx     one telemetry payload as received: t (server ms), dev, p
    est    one estimate: t, tag, x, y, vx, vy, sigma, b (range offset), used
    mark   operator marker: t, label, optional x/y of a known point
"""
import json
import os
import re
import time


class Recorder:
    def __init__(self, directory):
        self.dir = directory
        self.fh = None
        self.path = None
        self.started = None
        self.lines = 0

    @property
    def active(self):
        return self.fh is not None

    def start(self, name, note, scene, extra=None, t=None):
        self.stop()
        os.makedirs(self.dir, exist_ok=True)
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "-", name or "").strip("-") or "sesi"
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.path = os.path.join(self.dir, f"{stamp}_{safe}.jsonl")
        self.fh = open(self.path, "w", encoding="utf-8")
        self.started = int(time.time() * 1000) if t is None else int(t)
        self.lines = 0
        self.write({"k": "meta", "t": self.started, "name": name, "note": note,
                    "scene": scene, **(extra or {})})
        return self.path

    def stop(self):
        if self.fh:
            self.fh.close()
        self.fh = None
        path, self.path = self.path, None
        return path

    def write(self, obj):
        if not self.fh:
            return
        self.fh.write(json.dumps(obj, separators=(",", ":")) + "\n")
        self.lines += 1
        if self.lines % 50 == 0:
            self.fh.flush()

    def status(self):
        return {"active": self.active,
                "file": os.path.basename(self.path) if self.path else None,
                "started": self.started if self.active else None,
                "lines": self.lines if self.active else 0}

    def list(self):
        if not os.path.isdir(self.dir):
            return []
        out = []
        for f in sorted(os.listdir(self.dir), reverse=True):
            if f.endswith(".jsonl"):
                p = os.path.join(self.dir, f)
                out.append({"file": f, "size": os.path.getsize(p),
                            "mtime": int(os.path.getmtime(p))})
        return out
