"""Research engine: server/tracker.py with every value of the studies adjustable.

The server runs one fixed algorithm (server/tracker.py). The simulation and
replay tools compare it with the variants it was chosen against, and those
need the knobs: windows on the ranges before the EKF and on the position after
it, extrapolation, the older still-averaging, offsets learned or not, the
zero-velocity mode on or off. They live here so the server code stays the
algorithm only.

    eng = LabEngine()
    eng.configure({... "tracking": {"post_filter": "still", ...}}, ...)

LAB_DEFAULTS is algorithm G (the server before the standing-still work); the
final algorithm is  zero_velocity=1, still_long=1.0, post_filter="none"
(or "mean" with post_window=3 for the smoothed path), and with a static
calibration also estimate_bias=0, anchor_offsets=0.
"""
import bisect
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "server"))
import tracker  # noqa: E402

LAB_DEFAULTS = dict(tracker.PARAMS)
LAB_DEFAULTS.update({
    # 1 = estimate the range offset shared by all anchors online (needs 3+).
    "estimate_bias": 1.0,
    # 1 = also estimate each anchor's own offset d_i online.
    "anchor_offsets": 1.0,
    # The reported position may be predicted along the velocity from the
    # newest sample to "now" by at most this much (ms). Off: on the 23-10
    # recordings it nearly doubled the jumps > 40 cm.
    "extrapolate_ms": 0,
    # Stage 1, on the ranges: each anchor's last pre_window samples reduced to
    # one value read at the newest sample: "none", "median", "mean" or
    # "linfit" (least-squares line read at its newest end).
    "pre_filter": "none",
    "pre_window": 5,
    # Stage 2, on the reported position: the last post_window outputs with the
    # same choices, plus "still": the outputs since the trolley stopped (EKF
    # speed below still_speed, moving again above twice it), up to post_window.
    "post_filter": "still",
    "post_window": 20,
    "still_speed": 0.15,
    # Algorithm G had neither of these.
    "zero_velocity": 0.0,
    "still_long": 0.0,
})
WINDOW_FILTERS = ("none", "median", "mean", "linfit")
CHOICES = {"pre_filter": WINDOW_FILTERS, "post_filter": WINDOW_FILTERS + ("still",)}


def tracking_params(scene):
    """LAB_DEFAULTS overridden by scene["tracking"]."""
    out = dict(LAB_DEFAULTS)
    raw = (scene or {}).get("tracking") or {}
    for k in out:
        if k not in raw:
            continue
        if k in CHOICES:
            if str(raw[k]) in CHOICES[k]:
                out[k] = str(raw[k])
            continue
        try:
            out[k] = float(raw[k])
        except (TypeError, ValueError):
            pass
    return out


def window_value(samples, kind):
    """One value from a window of (t_ms, value) pairs, oldest first, read at
    the newest sample."""
    return window_fit(samples, kind)[0]


def window_fit(samples, kind):
    """(value at the newest sample, slope per second) of a window. The slope
    is only estimated by "linfit" (0 otherwise)."""
    vals = [v for _t, v in samples]
    n = len(vals)
    if kind == "none" or n == 1:
        return vals[-1], 0.0
    if kind == "median":
        return tracker.median(vals), 0.0
    mean = sum(vals) / n
    if kind == "mean" or n < 3:
        return mean, 0.0
    # "linfit": v = mean + slope * (t - t_mean), evaluated at the newest t
    t_new = samples[-1][0]
    ts = [(t - t_new) / 1000.0 for t, _v in samples]
    tm = sum(ts) / n
    stt = sum((t - tm) ** 2 for t in ts)
    if stt < 1e-9:
        return mean, 0.0
    slope = sum((t - tm) * (v - mean) for t, v in zip(ts, vals)) / stt
    return mean - slope * tm, slope


class LabEngine(tracker.Engine):
    """tracker.Engine with the settings of tracking_params() (see module doc)."""

    def __init__(self):
        super().__init__()
        self.params = dict(LAB_DEFAULTS)

    def _load_params(self, scene):
        return tracking_params(scene)

    def _offset_policy(self):
        p = self.params
        return p["estimate_bias"] >= 0.5 and len(self.anchors) >= 3, p["anchor_offsets"] >= 0.5

    def _standing_options(self):
        return self.params["zero_velocity"] >= 0.5, self.params["still_long"]

    def _still(self, tr):
        return tr.still_from is not None

    def _range_value(self, tr, anchor, t, z):
        """Stage 1: put the range in this anchor's window and return the value
        the EKF uses (pre_filter over the last pre_window samples)."""
        p = self.params
        buf = tr.win.setdefault(anchor, [])
        if buf and t - buf[-1][0] > p["window_ms"]:
            buf.clear()                       # a gap: do not average across it
        bisect.insort(buf, (t, z))
        keep = max(int(p["pre_window"]), tracker.BOOT_SAMPLES)
        if len(buf) > keep:
            del buf[:len(buf) - keep]
        n = max(int(p["pre_window"]), 1)
        return window_value(buf[-n:], p["pre_filter"])

    def _reported(self, tr, px, py, vx, vy, age):
        """Stage 2: the position from the last post_window filter outputs, then
        extrapolated along the velocity ("linfit": the slope of the fitted
        line, else the EKF's) by at most extrapolate_ms."""
        p = self.params
        t = tr.f.t
        win = tr.post
        if not win or t > win[-1][0]:
            win.append((t, px, py))
        n = max(int(p["post_window"]), 1)
        while win and (len(win) > n or t - win[0][0] > p["window_ms"]):
            del win[0]
        kind = p["post_filter"]
        if kind == "none":
            fx, fy, fvx, fvy = px, py, vx, vy
        elif kind == "still":
            speed = math.hypot(vx, vy)
            if tr.still_from is not None and speed > 2.0 * p["still_speed"]:
                tr.still_from = None              # moving again
            elif tr.still_from is None and speed < p["still_speed"]:
                tr.still_from = t                 # stopped: start averaging
            if tr.still_from is None:
                fx, fy, fvx, fvy = px, py, vx, vy
            else:
                pts = [(x, y) for tt, x, y in win if tt >= tr.still_from]
                fx = sum(q[0] for q in pts) / len(pts)
                fy = sum(q[1] for q in pts) / len(pts)
                fvx = fvy = 0.0
        else:
            fx, fvx = window_fit([(tt, x) for tt, x, _y in win], kind)
            fy, fvy = window_fit([(tt, y) for tt, _x, y in win], kind)
            if kind != "linfit":
                fvx, fvy = vx, vy
        ahead = min(age, p["extrapolate_ms"]) / 1000.0
        return fx + fvx * ahead, fy + fvy * ahead, fvx, fvy
