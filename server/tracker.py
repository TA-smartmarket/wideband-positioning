"""Tag tracking engine — measurement bookkeeping + EKF on slant ranges.

The algorithm is CALF-UWB (Correction-Aware Layered Filter for UWB Indoor
Positioning): range correction from the static calibration (K), a robust EKF
(G), the standing-still constraints (Z, L) and a validity gate with optional
smoothing. Points 6-10 below; docs/ALGORITHMS.md has the details.

Why this module exists
----------------------
The first server re-ran the filter on *every* incoming message and fed it
*every range it still had stored* (anything younger than 5 s). One telemetry
packet per anchor and per tag, plus the MQTT per-range messages, meant the same
physical measurement was applied to the EKF five to ten times as if each copy
were new evidence. Three things followed:

* the covariance collapsed (the filter believed it was accurate to a few
  millimetres), so every real movement looked like an outlier;
* ranges taken at different moments were fused as if simultaneous, so the
  estimate was pulled along one anchor's circle, then back along another's —
  the zig-zag that shows up as soon as the tag moves;
* the innovation gate then rejected good data until the 8-cycle self-heal
  re-initialised the track, which is the sudden jump after a straight walk.

The rules here are the textbook ones that were missing:

1. **Every measurement is used exactly once.** Samples are deduplicated per
   link and per reporting device by their device timestamp, so the old
   firmware (which re-sends its latest range every second) is handled too.
2. **Every measurement is applied at the time it was taken.** Device clocks
   are mapped to server time (minimum-latency offset per device) and the
   filter predicts to that instant before the update.
3. **One source per link.** The tag and the anchor both report the same
   two-way-ranging result; the tag's copy is used whenever the tag reports
   that link itself (all its ranges share one clock), the anchor's otherwise.
4. **The measurement model is the slant range.** The anchor hangs above the
   tag, so  h(x) = sqrt((px-ax)^2 + (py-ay)^2 + dz^2)  with dz known. The
   previous projection sqrt(r^2 - dz^2) amplified noise near an anchor and
   collapsed to a bogus 0.1*r whenever noise made r < dz.
5. **Robust update instead of a hard gate.** Large innovations are
   down-weighted (Huber); only gross outliers are rejected, and if nothing has
   been accepted for a while the track is re-initialised from multilateration.
6. **Per-anchor range calibration** (bias + scale) measured with the tag at
   known points — see /api/v1/calibration in app.py.
7. **Range offsets from the calibration, or learned.** With a static
   calibration of every anchor the offsets are taken from it and kept; without
   one the shared offset b and each anchor's d_i are learned online.
8. **No position from too little data.** A position is valid only while 3+
   anchors have recent accepted ranges; otherwise the last valid position is
   held and flagged invalid instead of a guess from one or two anchors.
9. **Standing still.** A zero-velocity mode pins the velocity to 0 while the
   trolley stands, and while it stands a range that came in too long is
   dropped (a shopper between tag and anchor only ever lengthens a range).
10. **Optional smoothing** of the path shown: the mean of the last 3 positions.

The algorithm is fixed (docs/ALGORITHMS.md; chosen by simulation in
docs/hasil-simulasi/RISET_JENDELA.md); only the smoothing can be switched.
The research variants it was compared against — windows on the ranges or the
position, other offset handling, tunable values — are in tools/tracker_lab.py.

Everything here is plain Python (no numpy) so the server keeps its two
dependencies, and the same code runs in tools/simulate.py and
tools/evaluate.py for offline replay.
"""

import bisect
import math

# ---------------------------------------------------------------------------
# tuning
# ---------------------------------------------------------------------------
PARAMS = {
    # White-acceleration process noise (m/s^2). A trolley changes speed
    # gently; 0.8 keeps straight runs straight while still following a
    # 90-degree turn without overshooting the corner.
    "sigma_a": 0.8,
    # LOS range noise of one DW1000 two-way-ranging sample (m).
    "sigma_r": 0.12,
    # Normalised innovation |y|/sqrt(S) where down-weighting starts (Huber).
    "huber_k": 2.5,
    # Same, for ranges LONGER than expected. A body or wall in the path only
    # ever makes a UWB range longer, so a long range is more suspicious than a
    # short one and is down-weighted earlier.
    "huber_k_long": 1.0,
    # Normalised innovation beyond which a sample is rejected outright.
    "gate": 6.0,
    # A sample older than the filter state by more than this is dropped.
    "max_late_ms": 400,
    # No accepted sample for this long -> re-initialise from multilateration.
    "reinit_ms": 1500,
    # How far outside the room rectangle the estimate may go (m).
    "room_margin": 0.5,
    # Velocity correlation time (s), Singer / Ornstein-Uhlenbeck model: across
    # a WiFi stall of a few seconds it stops the estimate from coasting on a
    # noisy velocity (seen: 45 cm of drift for a tag lying still).
    "vel_tau_s": 1.0,

    # --- validity -------------------------------------------------------------
    # The position is valid only while at least min_anchors different anchors
    # had an accepted range within fresh_ms of the filter time and the
    # filter's sigma is below valid_sigma (m). Otherwise the last valid
    # position is held and flagged invalid: one or two anchors cannot fix a
    # 2-D position (seen on hardware: 2.4 m sigma, a track pushed out of the room).
    "min_anchors": 3,
    "fresh_ms": 1500,
    "valid_sigma": 0.5,

    # --- standing still ---------------------------------------------------------
    # Zero-velocity mode: once the EKF speed has stayed below 0.2 m/s for
    # 0.4 s the trolley is taken to stand: its velocity is pinned to 0 and the
    # process noise made small, so one bad round of ranges can no longer move
    # the position; it ends as soon as 3 of the last 4 ranges disagree with
    # the position by more than 1.5 sigma, or 2 of the last 10 come in more
    # than 1 sigma SHORTER than expected (a body only lengthens a range).
    "zero_velocity": 1.0,
    # While standing, a range LONGER than expected by more than this (sigma)
    # is dropped instead of down-weighted, as long as the other anchors can
    # still fix the position.
    "still_long": 1.0,

    # --- the path shown -----------------------------------------------------------
    # 1 = the mean of the last smooth_window positions (one per filter step,
    # never across a gap longer than window_ms): a smoother line, ~0.35 s
    # more lag while driving. The only setting (Setup -> Tracker).
    "smooth": 1.0,
    "smooth_window": 3,
    "window_ms": 4000,
}

# Anchor copies of a link are used only when the tag itself has sent nothing
# for this long (ms). The tag's WiFi uplink stalls for ~3 s now and then and
# delivers the backlog afterwards, so a short hold switched to the anchors'
# one-link-at-a-time batches exactly when they do the most damage.
SOURCE_HOLD_MS = 10000
# Fresh-sample window used for bootstrapping / re-initialisation (ms).
FIX_WINDOW_MS = 700
# Samples kept per anchor at least (the bootstrap takes their median).
BOOT_SAMPLES = 5


def tracking_params(scene):
    """The algorithm's values and its one setting, scene["tracking"]["smooth"]."""
    out = dict(PARAMS)
    raw = (scene or {}).get("tracking") or {}
    if "smooth" in raw:
        try:
            out["smooth"] = 1.0 if float(raw["smooth"]) >= 0.5 else 0.0
        except (TypeError, ValueError):
            pass
    return out


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

def median(vals):
    v = sorted(vals)
    n = len(v)
    m = n // 2
    return v[m] if n % 2 else 0.5 * (v[m - 1] + v[m])


def horizontal_range(range3d, dz):
    """Slant range -> horizontal range (only used for the closed-form guess)."""
    if range3d <= 0:
        return 0.0
    r2 = range3d * range3d - dz * dz
    return math.sqrt(r2) if r2 > 0 else 0.0


def geometry_check(fixes):
    """Triangle-inequality test on (x, y, horizontal_range) triples.

    |r1 - r2| <= d <= r1 + r2 must hold for every anchor pair; a violation
    means at least one range is wrong (typically NLOS: the reflection is
    longer than the direct path). Returns (ok, worst_slack_m).
    """
    worst = 0.0
    for i in range(len(fixes)):
        for j in range(i + 1, len(fixes)):
            (x1, y1, r1), (x2, y2, r2) = fixes[i], fixes[j]
            d = math.hypot(x2 - x1, y2 - y1)
            if d < 1e-6:
                continue
            if r1 + r2 < d:
                worst = max(worst, d - (r1 + r2))
            elif abs(r1 - r2) > d:
                worst = max(worst, abs(r1 - r2) - d)
    return worst <= 0.0, worst


def solve_2d(fixes, room_w, room_h):
    """Closed-form fix from (x, y, horizontal_range) triples.

    2 anchors: intersection of two circles, the mirror image is resolved with
    the room rectangle. 3+: linearised least squares + Gauss-Newton.
    """
    if len(fixes) < 2:
        return None
    use_room = room_w > 0 and room_h > 0

    if len(fixes) == 2:
        (x1, y1, r1), (x2, y2, r2) = fixes
        dx, dy = x2 - x1, y2 - y1
        d = math.hypot(dx, dy)
        if d < 1e-4:
            return None
        if d > r1 + r2 or d < abs(r1 - r2):
            # inconsistent pair: best point on the baseline
            t = min(max((r1 - r2 + d) / (2 * d), 0.0), 1.0)
            return {"x": x1 + t * dx, "y": y1 + t * dy, "ambiguous": True}
        aa = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
        hh = math.sqrt(max(r1 * r1 - aa * aa, 0.0))
        mx, my = x1 + aa * dx / d, y1 + aa * dy / d
        c = [(mx + hh * dy / d, my - hh * dx / d),
             (mx - hh * dy / d, my + hh * dx / d)]
        if use_room:
            inside = [pt for pt in c if 0 <= pt[0] <= room_w and 0 <= pt[1] <= room_h]
            if len(inside) == 1:
                return {"x": inside[0][0], "y": inside[0][1], "ambiguous": False}
        cx, cy = room_w / 2, room_h / 2
        pt = min(c, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
        return {"x": pt[0], "y": pt[1], "ambiguous": True}

    (x0, y0, r0) = fixes[0]
    A0 = A1 = B0 = B1 = C = 0.0
    for (xi, yi, ri) in fixes[1:]:
        ax, ay = 2.0 * (xi - x0), 2.0 * (yi - y0)
        b = (ri * ri - r0 * r0) - xi * xi + x0 * x0 - yi * yi + y0 * y0
        A0 += ax * ax; A1 += ax * ay; B0 += ax * b; B1 += ay * b; C += ay * ay
    det = A0 * C - A1 * A1
    if abs(det) < 1e-9:
        return None
    # The linear system solves for -p (sign of the 2(xi-x0) rows), hence the minus.
    px = -(B0 * C - B1 * A1) / det
    py = -(A0 * B1 - A1 * B0) / det
    return {"x": px, "y": py, "ambiguous": False}


def multilaterate(anchor_fixes, tag_z, room_w, room_h, iters=6):
    """Position from slant ranges: closed-form guess + Gauss-Newton on the
    3D model  r_i = sqrt((x-ax)^2 + (y-ay)^2 + (tag_z-az)^2).

    anchor_fixes: list of (ax, ay, az, slant_range). Returns dict with x, y,
    rms (m, residual of the slant model) or None.
    """
    if len(anchor_fixes) < 2:
        return None
    flat = [(ax, ay, horizontal_range(r, az - tag_z)) for ax, ay, az, r in anchor_fixes]
    guess = solve_2d(flat, room_w, room_h)
    if guess is None:
        return None
    px, py = guess["x"], guess["y"]
    if len(anchor_fixes) >= 3:
        for _ in range(iters):
            j00 = j01 = j11 = e0 = e1 = 0.0
            for ax, ay, az, r in anchor_fixes:
                dx, dy, dz = px - ax, py - ay, tag_z - az
                rho = math.sqrt(dx * dx + dy * dy + dz * dz) or 1e-6
                res = rho - r
                hx, hy = dx / rho, dy / rho
                j00 += hx * hx; j01 += hx * hy; j11 += hy * hy
                e0 += hx * res; e1 += hy * res
            det = j00 * j11 - j01 * j01
            if abs(det) < 1e-9:
                break
            sx = (j11 * e0 - j01 * e1) / det
            sy = (j00 * e1 - j01 * e0) / det
            px -= sx
            py -= sy
            if abs(sx) + abs(sy) < 1e-4:
                break
    ss = 0.0
    for ax, ay, az, r in anchor_fixes:
        rho = math.sqrt((px - ax) ** 2 + (py - ay) ** 2 + (tag_z - az) ** 2)
        ss += (rho - r) ** 2
    return {"x": px, "y": py, "rms": math.sqrt(ss / len(anchor_fixes)),
            "ambiguous": guess.get("ambiguous", False) and len(anchor_fixes) == 2}


def multilaterate_with_offset(anchor_fixes, tag_z, room_w, room_h, lo=-0.5, hi=1.5):
    """Position AND the offset b shared by every range, for a cold start
    (antenna-delay offset of ~0.75-1 m not known yet). With 4+ anchors there
    is one equation more than x, y, b. b is found by a 1-D search: for each
    candidate b the ranges are corrected and the plain fix computed; the b
    with the smallest residual wins, refined to 1 cm. A search, not a joint
    Gauss-Newton: that diverged (b = 3 m) in a 2.6 x 1.3 m room with all
    anchors at one height. Returns the multilaterate() dict plus "b"."""
    def fix(b):
        sol = multilaterate([(ax, ay, az, r - b) for ax, ay, az, r in anchor_fixes],
                            tag_z, room_w, room_h)
        return (sol["rms"], b, sol) if sol else None

    best = None
    for k in range(int(round((hi - lo) / 0.05)) + 1):
        cand = fix(lo + 0.05 * k)
        if cand and (best is None or cand[0] < best[0]):
            best = cand
    if best is None:
        return None
    for k in range(-5, 6):
        cand = fix(best[1] + 0.01 * k)
        if cand and cand[0] < best[0]:
            best = cand
    out = dict(best[2])
    out["b"] = best[1]
    return out


# ---------------------------------------------------------------------------
# device clock -> server clock
# ---------------------------------------------------------------------------

class ClockSync:
    """Maps a device's millis() to server milliseconds.

    offset = min over packets of (receive_time - device_time), i.e. the
    fastest delivery seen so far. Network jitter only ever makes a packet
    *later*, so the minimum is the best estimate of the true offset plus a
    small constant latency. The offset may creep upward slowly (200 ppm) to
    follow clock drift, and is reset when the device reboots (millis() goes
    backwards).
    """

    CREEP = 2e-4

    def __init__(self):
        self.offset = None
        self.last_dev = None
        self.last_recv = None

    def observe(self, dev_ms, recv_ms):
        """Returns True when the device has rebooted (millis() went back)."""
        cand = recv_ms - dev_ms
        rebooted = self.last_dev is not None and dev_ms < self.last_dev - 2000
        if self.offset is None or self.last_dev is None or rebooted:
            self.offset = cand
        else:
            creep = max(recv_ms - self.last_recv, 0) * self.CREEP
            self.offset = min(self.offset + creep, cand)
        self.last_dev = dev_ms
        self.last_recv = recv_ms
        return rebooted

    def to_server(self, dev_ms):
        return dev_ms + (self.offset or 0.0)


# ---------------------------------------------------------------------------
# EKF
#
#   state  x = [px, py, vx, vy, b]
#            b = range offset shared by every anchor (m)
#   motion constant velocity + white acceleration; b is a slow random walk
#   model  r_i = sqrt((px-ax)^2 + (py-ay)^2 + dz^2) + b
#
# Why b: an uncalibrated DW1000 antenna delay adds the same few tens of
# centimetres to every range of a tag. With 4 anchors there are two more
# ranges than unknowns (px, py), so that common offset is observable and the
# filter can estimate it online — exactly like the receiver clock bias in GPS.
# It is only enabled with 3+ anchors; with 2 it would not be observable.
# With a static calibration of every anchor, b and the d_i below are not
# estimated: the ranges are already corrected by the calibration.
# ---------------------------------------------------------------------------

N_STATE = 5
BIAS_SIGMA0 = 0.3          # prior on b (m)
BIAS_WALK = 0.003          # random walk of b (m / sqrt(s))
BIAS_LIMIT = (-0.5, 1.5)   # physically plausible range of b (m)
# Per-anchor offsets d_i on top of b (without a calibration): each
# anchor's own antenna-delay difference. Not observable from one spot (4
# ranges, 7 unknowns) but observable once the trolley moves through the room
# and sees every anchor from different directions: online self-calibration.
# A tight prior keeps them near 0 until the data says more.
ANCHOR_OFF_SIGMA0 = 0.10   # prior on d_i (m): spread between DW1000 modules
ANCHOR_OFF_WALK = 0.0005   # random walk of d_i (m / sqrt(s))
ANCHOR_OFF_LIMIT = 0.30    # |d_i| bound (m)
# Zero-velocity mode: when it starts and ends, and what the filter assumes
# meanwhile.
ZUPT_SLOW, ZUPT_AFTER_MS = 0.20, 400     # enter: speed < 0.2 m/s for 0.4 s
ZUPT_SIGMA_A, ZUPT_SIGMA_V = 0.05, 0.02  # standing: m/s^2 of process noise, m/s on v = 0
ZUPT_EXIT_NU, ZUPT_EXIT_N = 1.5, 3       # leave: 3 of the last 4 |nu| > 1.5
# ... or 2 of the last 10 ranges (~0.8 s) nu < -1. A body or a blocked path only
# lengthens a range, so a range clearly SHORTER than expected means the tag
# moved towards that anchor. Setting off diagonally between two anchors
# hardly changes their ranges and the anchor behind is dropped as "a body",
# so "3 of the last 4" alone held the position for up to 3 s while the tag
# had moved 1.3 m (tools/room_walk.py). Any anchor counts: the one ahead may
# miss a few rounds.
ZUPT_EXIT_SHORT, ZUPT_EXIT_SHORT_N, ZUPT_EXIT_SHORT_OF = 1.0, 2, 10
# Not standing again for 1 s after leaving: when the anchors ahead miss a few
# rounds the speed has not grown yet, and the mode came back while the tag
# drove on.
ZUPT_HOLD_MS = 1000


class TagFilter:
    def __init__(self, px, py, t_ms, sigma_a, sigma_pos=0.5, bias=0.0,
                 estimate_bias=True, vel_tau=0.0, offsets=None,
                 zero_velocity=False, still_long=0.0):
        """offsets: initial per-anchor offsets d_i (one per anchor index) to
        add to the state; None keeps the state [px, py, vx, vy, b].
        zero_velocity / still_long: see PARAMS."""
        self.n_off = len(offsets) if offsets else 0
        n = N_STATE + self.n_off
        self.x = [px, py, 0.0, 0.0, bias] + list(offsets or [])
        self.P = [[0.0] * n for _ in range(n)]
        self.P[0][0] = self.P[1][1] = sigma_pos * sigma_pos
        self.P[2][2] = self.P[3][3] = 1.0
        self.P[4][4] = BIAS_SIGMA0 ** 2 if estimate_bias else 0.0
        for i in range(self.n_off):
            self.P[N_STATE + i][N_STATE + i] = ANCHOR_OFF_SIGMA0 ** 2
        self.estimate_bias = estimate_bias
        self.t = t_ms
        self.sigma_a = sigma_a
        self.vel_tau = vel_tau
        self.zero_velocity = zero_velocity
        self.still_long = still_long
        self.standing = False         # in the zero-velocity mode
        self.drop_ok = True           # set by the engine: enough other anchors for still_long
        self.slow_since = None
        self.recent = []              # standing: |nu| > ZUPT_EXIT_NU of the last updates
        self.short = []               # standing: nu < -ZUPT_EXIT_SHORT of the last updates
        self.left_at = None           # when the zero-velocity mode last ended

    def predict(self, t_ms):
        dt = (t_ms - self.t) / 1000.0
        if dt <= 0:
            return
        self.t = t_ms
        dt = min(dt, 2.0)
        n, x, P = len(self.x), self.x, self.P
        # velocity decay a and displacement per unit velocity g
        # (Singer model; tau -> infinity gives constant velocity: a=1, g=dt)
        if self.vel_tau > 0:
            a = math.exp(-dt / self.vel_tau)
            g = self.vel_tau * (1.0 - a)
        else:
            a, g = 1.0, dt
        x[0] += x[2] * g
        x[1] += x[3] * g
        x[2] *= a
        x[3] *= a
        # P = F P F^T + Q,  F = [[1,0,g,0],[0,1,0,g],[0,0,a,0],[0,0,0,a]] (+ b)
        FP = [list(row) for row in P]
        for j in range(n):
            FP[0][j] += g * P[2][j]
            FP[1][j] += g * P[3][j]
            FP[2][j] = a * P[2][j]
            FP[3][j] = a * P[3][j]
        FPF = [list(row) for row in FP]
        for i in range(n):
            FPF[i][0] += g * FP[i][2]
            FPF[i][1] += g * FP[i][3]
            FPF[i][2] = a * FP[i][2]
            FPF[i][3] = a * FP[i][3]
        q = (ZUPT_SIGMA_A if self.standing else self.sigma_a) ** 2
        dt2 = dt * dt
        q11, q13, q33 = q * dt2 * dt2 / 4.0, q * dt2 * dt / 2.0, q * dt2
        FPF[0][0] += q11; FPF[1][1] += q11
        FPF[0][2] += q13; FPF[2][0] += q13
        FPF[1][3] += q13; FPF[3][1] += q13
        FPF[2][2] += q33; FPF[3][3] += q33
        if self.estimate_bias:
            FPF[4][4] += BIAS_WALK * BIAS_WALK * dt
        for i in range(N_STATE, n):
            FPF[i][i] += ANCHOR_OFF_WALK * ANCHOR_OFF_WALK * dt
        self.P = FPF
        # keep sigma interpretable during long outages
        for i, cap in ((0, 25.0), (1, 25.0), (2, 4.0), (3, 4.0)):
            if self.P[i][i] > cap:
                s = math.sqrt(cap / self.P[i][i])
                for j in range(n):
                    self.P[i][j] *= s
                    self.P[j][i] *= s
        if self.standing:                     # zero-velocity pseudo-measurement
            self._direct(2, 0.0, ZUPT_SIGMA_V ** 2)
            self._direct(3, 0.0, ZUPT_SIGMA_V ** 2)

    def _direct(self, j, z, R):
        """Scalar update of state j measured directly as z (variance R)."""
        x, P = self.x, self.P
        n = len(x)
        S = P[j][j] + R
        K = [P[i][j] / S for i in range(n)]
        y = z - x[j]
        for i in range(n):
            x[i] += K[i] * y
        self.P = [[P[i][q] - K[i] * P[j][q] for q in range(n)] for i in range(n)]

    def _standing_mode(self, nu):
        """Enter / leave the zero-velocity mode after each range."""
        if not self.zero_velocity:
            return
        if self.standing:
            self.recent = (self.recent + [abs(nu) > ZUPT_EXIT_NU])[-4:]
            self.short = (self.short + [nu < -ZUPT_EXIT_SHORT])[-ZUPT_EXIT_SHORT_OF:]
            if sum(self.recent) >= ZUPT_EXIT_N or sum(self.short) >= ZUPT_EXIT_SHORT_N:
                # the ranges keep moving, or the tag came closer to an anchor: it goes
                self.standing, self.slow_since, self.recent, self.short = False, None, [], []
                self.left_at = self.t
                self.P[2][2] += 0.36              # let the velocity grow again
                self.P[3][3] += 0.36
                self.P[0][0] += 0.01
                self.P[1][1] += 0.01
            return
        if self.left_at is not None and self.t - self.left_at < ZUPT_HOLD_MS:
            self.slow_since = None
        elif math.hypot(self.x[2], self.x[3]) < ZUPT_SLOW:
            if self.slow_since is None:
                self.slow_since = self.t
            elif self.t - self.slow_since >= ZUPT_AFTER_MS:
                self.standing, self.recent, self.short = True, [], []
        else:
            self.slow_since = None

    def predicted_range(self, ax, ay, dz):
        dx, dy = self.x[0] - ax, self.x[1] - ay
        rho = math.sqrt(dx * dx + dy * dy + dz * dz) or 1e-6
        return rho, (dx / rho, dy / rho)

    def update(self, ax, ay, dz, z, sigma, huber_k, gate, k=None, huber_k_long=0.0):
        """One slant-range update from anchor index k (only used with
        per-anchor offsets). Returns (accepted, y, nu, S)."""
        n, P = len(self.x), self.P
        rho, (h0, h1) = self.predicted_range(ax, ay, dz)
        hb = 1.0 if self.estimate_bias else 0.0
        H = [0.0] * n
        H[0], H[1], H[4] = h0, h1, hb
        if k is not None and k < self.n_off:
            H[N_STATE + k] = 1.0
        nz = [j for j in range(n) if H[j]]
        y = z - (rho + sum(H[j] * self.x[j] for j in nz if j >= 4))
        PHt = [sum(P[i][j] * H[j] for j in nz) for i in range(n)]
        HPHt = sum(H[j] * PHt[j] for j in nz)
        R = sigma * sigma
        S = HPHt + R
        nu = y / math.sqrt(S)
        self._standing_mode(nu)
        if abs(nu) > gate:
            return False, y, nu, S
        if self.standing and self.still_long > 0 and nu > self.still_long and self.drop_ok:
            return False, y, nu, S            # standing: too long = a body in the way
        kh = huber_k_long if (nu > 0 and huber_k_long > 0) else huber_k
        if abs(nu) > kh:                      # Huber: inflate R, do not reject
            R *= abs(nu) / kh
            S = HPHt + R
        K = [PHt[i] / S for i in range(n)]
        for i in range(n):
            self.x[i] += K[i] * y
        # Joseph form: P = (I-KH) P (I-KH)^T + K R K^T  (stays symmetric PSD)
        IKH = [[(1.0 if i == j else 0.0) - K[i] * H[j] for j in range(n)] for i in range(n)]
        A = [[sum(IKH[i][k] * P[k][j] for k in range(n)) for j in range(n)] for i in range(n)]
        self.P = [[sum(A[i][k] * IKH[j][k] for k in range(n)) + K[i] * R * K[j]
                   for j in range(n)] for i in range(n)]
        lo, hi = BIAS_LIMIT
        self.x[4] = min(max(self.x[4], lo), hi)
        for i in range(N_STATE, n):
            self.x[i] = min(max(self.x[i], -ANCHOR_OFF_LIMIT), ANCHOR_OFF_LIMIT)
        return True, y, nu, S

    def sigma(self):
        return math.sqrt(max(self.P[0][0], 0.0) + max(self.P[1][1], 0.0))


class TagTrack:
    def __init__(self, tag_id):
        self.id = tag_id
        self.f = None                 # TagFilter once initialised
        self.latest = {}              # anchor -> (t, slant_range_corrected, raw)
        self.links = {}               # anchor -> diagnostics for the UI
        self.last_accept = None
        self.accepted_ts = []         # recent accepted sample times (rate)
        self.sample_ts = []           # recent sample times (rate)
        self.reinits = 0
        self.dropped_late = 0
        self.bias = 0.0               # last common-offset estimate, kept across re-inits
        self.offsets = {}             # anchor -> last per-anchor offset d_i (same)
        self.bias_known = False       # b solved at a fix or learned by the EKF
        self.implausible = 0          # samples dropped as physically impossible
        self.first_t = None           # first sample seen (bootstrap patience)
        self.warm = 0                 # accepted samples since the last (re)init
        self.win = {}                 # anchor -> [(t, range)] newest ranges (bootstrap median)
        self.acc_t = {}               # anchor -> time of its last accepted range
        self.post = []                # [(filter t, x, y)] last outputs (smoothing)
        self.still_from = None        # used only by tools/tracker_lab.py (post_filter "still")
        self.hold = None              # last valid (x, y), reported while invalid

    def reset_output(self):
        self.post.clear()
        self.still_from = None


class Engine:
    """Owns every tag track. Not thread-safe: the caller holds its lock."""

    def __init__(self):
        self.scene = {"room": {"width": 5.0, "depth": 4.0}, "anchors": [],
                      "obstacles": [], "tag_z": 1.3}
        self.nlos_enabled = True
        self.anchors = {}             # id -> (x, y, z)
        self.cal = {}                 # id -> (bias, scale)
        self.calibrated = False       # every anchor has a static calibration
        self.params = dict(PARAMS)
        self.tracks = {}              # tag -> TagTrack
        self.clocks = {}              # reporting device -> ClockSync
        self.last_dev_ts = {}         # (reporter, anchor, tag) -> device ts consumed
        self.tag_reported = {}        # (anchor, tag) -> server ms the tag last reported it
        self.on_sample = None         # optional callbacks (recorder)
        self.nlos_mod = None          # scene module (obstacle tests), injected

    # ----- configuration ---------------------------------------------------
    def configure(self, scene, nlos_enabled=True, extra_anchors=None, nlos_mod=None):
        """(Re)load geometry + calibration + the smoothing setting. Tracks are
        kept unless the caller also calls reset() (it should, when anchors
        move or the calibration changes)."""
        self.scene = scene
        self.nlos_enabled = nlos_enabled
        if nlos_mod is not None:
            self.nlos_mod = nlos_mod
        anchors = {}
        for aid, pos in (extra_anchors or {}).items():
            anchors[aid] = tuple(pos)
        for a in scene.get("anchors", []):
            anchors[a["id"]] = (float(a["x"]), float(a["y"]), float(a.get("z", 2.2)))
        self.anchors = anchors
        cal = {}
        for aid, c in ((scene.get("calibration") or {}).get("anchors") or {}).items():
            try:
                scale = float(c.get("scale", 1.0)) or 1.0
                cal[aid] = (float(c.get("bias", 0.0)), scale)
            except (TypeError, ValueError, AttributeError):
                continue
        self.cal = cal
        self.calibrated = bool(anchors) and all(aid in cal for aid in anchors)
        self.params = self._load_params(scene)
        index = {aid: i for i, aid in enumerate(sorted(anchors))}
        n_off = len(index) if self._offset_policy()[1] else 0
        if index != getattr(self, "anchor_index", None) or n_off != getattr(self, "n_off", 0):
            # the per-anchor states no longer line up: restart the filters
            for tr in self.tracks.values():
                tr.f = None
                tr.first_t = None
        self.anchor_index, self.n_off = index, n_off
        zero_velocity, still_long = self._standing_options()
        for tr in self.tracks.values():
            tr.reset_output()
            if tr.f is not None:
                tr.f.sigma_a = self.params["sigma_a"]
                tr.f.vel_tau = self.params["vel_tau_s"]
                tr.f.zero_velocity = zero_velocity
                tr.f.still_long = still_long
                if not zero_velocity:
                    tr.f.standing = False

    # The four places tools/tracker_lab.py (research variants) differs:
    def _load_params(self, scene):
        return tracking_params(scene)

    def _offset_policy(self):
        """(estimate the shared offset b, estimate each anchor's d_i): only
        without a calibration, and b only with 3+ anchors (observable)."""
        learn = not self.calibrated
        return learn and len(self.anchors) >= 3, learn

    def _standing_options(self):
        """(zero-velocity mode on, still_long in sigma)."""
        return self.params["zero_velocity"] >= 0.5, self.params["still_long"]

    def _still(self, tr):
        return tr.f.standing

    def reset(self):
        self.tracks.clear()
        self.last_dev_ts.clear()
        self.tag_reported.clear()

    def tag_z(self):
        try:
            return float(self.scene.get("tag_z", 1.3))
        except (TypeError, ValueError):
            return 1.3

    def _max_range(self, pos):
        """Longest range that can occur from this anchor: to the farthest
        room corner, plus room_margin, plus up to 1.5 m of range offset."""
        room = self.scene.get("room", {})
        w, d = room.get("width", 0), room.get("depth", 0)
        if w <= 0 or d <= 0:
            return 200.0
        m = self.params["room_margin"]
        dx = max(abs(pos[0] + m), abs(w + m - pos[0]))
        dy = max(abs(pos[1] + m), abs(d + m - pos[1]))
        dz = abs(pos[2] - self.tag_z())
        return math.sqrt(dx * dx + dy * dy + dz * dz) + 1.5

    def correct(self, anchor, raw):
        bias, scale = self.cal.get(anchor, (0.0, 1.0))
        return (raw - bias) / scale

    # ----- ingest ------------------------------------------------------------
    def ingest(self, reporter, payload, recv_ms):
        """Take one telemetry payload from `reporter` (e.g. "tag-1",
        "anchor-3"). Returns the set of tag ids that received new samples."""
        ranges = payload.get("ranges") or []
        if not ranges or not reporter:
            return set()
        clock = self.clocks.setdefault(reporter, ClockSync())
        # The packet's own timestamp is the tightest bound on the offset;
        # fall back to the newest sample timestamp for old payloads.
        dev_now = payload.get("ts")
        if not isinstance(dev_now, (int, float)) or dev_now <= 0:
            dev_now = max((r.get("ts") or 0) for r in ranges)
        if dev_now > 0 and clock.observe(float(dev_now), float(recv_ms)):
            # The reporter restarted, so its millis() began again near 0. Its
            # old "already used" marks would hide every new sample until the
            # new uptime passed the old one (seen on hardware after an OTA:
            # the tag was ignored for minutes).
            for k in [k for k in self.last_dev_ts if k[0] == reporter]:
                del self.last_dev_ts[k]

        samples = {}
        for r in ranges:
            src, dst = r.get("src"), r.get("dst")
            if not src or not dst:
                continue
            anchor = src if src.startswith("anchor") else dst
            tag = dst if src.startswith("anchor") else src
            if not tag.startswith("tag"):
                continue
            dev_ts = r.get("ts") or 0
            key = (reporter, anchor, tag)
            # exactly once: the old firmware re-sends its latest range with the
            # same device timestamp until a new one is measured
            if dev_ts and dev_ts <= self.last_dev_ts.get(key, -1):
                continue
            # One source per tag: while the tag itself is uplinking, the
            # anchors' copies are ignored altogether. An anchor batches only
            # its own link once a second, so mixing its copies in fed the
            # filter 3-5 updates from a single anchor in a row and pulled the
            # estimate toward it (decimetre jumps on hardware). The copies are
            # only a fallback for a tag whose WiFi uplink is down.
            if reporter == tag:
                self.tag_reported[tag] = recv_ms
            elif recv_ms - self.tag_reported.get(tag, -1e18) < SOURCE_HOLD_MS:
                continue
            if dev_ts:
                self.last_dev_ts[key] = dev_ts
            raw = r.get("raw")
            if not isinstance(raw, (int, float)):
                raw = r.get("range")
            if not isinstance(raw, (int, float)) or raw <= 0.01 or raw > 200:
                continue
            t = clock.to_server(float(dev_ts)) if dev_ts else float(recv_ms)
            # never in the future (clock offset not settled yet)
            t = min(t, float(recv_ms))
            samples.setdefault(tag, []).append(
                (t, anchor, float(raw), r.get("rx_power"), r.get("fp_power"), reporter))

        for tag, lst in samples.items():
            lst.sort(key=lambda s: s[0])
            tr = self.tracks.setdefault(tag, TagTrack(tag))
            for s in lst:
                if self.on_sample:
                    self.on_sample(tag, s)
                self._process(tr, *s)
        return set(samples)

    # ----- filter ------------------------------------------------------------
    def _fresh_fixes(self, tr, now):
        """(ax, ay, az, range) for every anchor heard in the last FIX_WINDOW_MS.
        The range is the median of that anchor's samples in the window, so a
        single spike cannot seed a wrong track."""
        out = []
        for anchor, buf in tr.win.items():
            pos = self.anchors.get(anchor)
            vals = [z for t, z in buf if now - FIX_WINDOW_MS <= t <= now]
            if pos and vals:
                out.append((pos[0], pos[1], pos[2], median(vals)))
        return out

    def _bootstrap(self, tr, now, force=False):
        fixes = self._fresh_fixes(tr, now)
        room = self.scene.get("room", {})
        # Seed from every anchor (up to 4): three that happened to answer first
        # can be nearly collinear with the tag and give a fix a metre off. Only
        # after 2 s of waiting (an anchor may be dead) is three enough.
        need = min(len(self.anchors), 4)
        if force or now - (tr.first_t or now) > 2000:
            need = min(need, 3)
        need = max(need, 2)
        if len(fixes) < need:
            return False
        # the common offset learned so far still applies to the new fix; on a
        # cold start (offset never learned) it is solved together with x, y
        est_b = self._offset_policy()[0]
        cold = est_b and not tr.bias_known
        fixes = [(ax, ay, az, r - tr.bias) for ax, ay, az, r in fixes]
        if (cold and len(fixes) < 4 <= len(self.anchors) and not force
                and now - (tr.first_t or now) <= 5000):
            return False                     # give 4 anchors 5 s, to learn b
        w, d = room.get("width", 0), room.get("depth", 0)
        sol = None
        if cold and len(fixes) >= 4:
            sol = multilaterate_with_offset(fixes, self.tag_z(), w, d, *BIAS_LIMIT)
            m = self.params["room_margin"]
            inside = sol and (w <= 0 or -m <= sol["x"] <= w + m) and (d <= 0 or -m <= sol["y"] <= d + m)
            if sol and inside and sol["rms"] <= 0.3:
                tr.bias = sol["b"]
                tr.bias_known = True
            else:
                sol = None                   # inconsistent: fall back to b as it is
        if sol is None:
            sol = multilaterate(fixes, self.tag_z(), w, d)
        if sol is None:
            return False
        # With 3+ anchors an inconsistent set (NLOS) would seed a wrong track;
        # wait for a better one unless we have been lost for a while.
        if len(fixes) >= 3 and sol["rms"] > 0.6 and not force:
            return False
        tr.warm = 0
        tr.reset_output()
        offsets = None
        if self.n_off and est_b:
            offsets = [0.0] * self.n_off
            for aid, i in self.anchor_index.items():
                offsets[i] = tr.offsets.get(aid, 0.0)
        zero_velocity, still_long = self._standing_options()
        tr.f = TagFilter(sol["x"], sol["y"], now, self.params["sigma_a"], bias=tr.bias,
                         estimate_bias=est_b, vel_tau=self.params["vel_tau_s"],
                         offsets=offsets, zero_velocity=zero_velocity, still_long=still_long)
        tr.last_accept = now
        tr.reinits += 1 if force else 0
        return True

    def _range_value(self, tr, anchor, t, z):
        """Keep the range in this anchor's short window (the bootstrap takes its
        median) and return the value the EKF uses: the anchor's newest range."""
        buf = tr.win.setdefault(anchor, [])
        if buf and t - buf[-1][0] > self.params["window_ms"]:
            buf.clear()                       # a gap: start again
        bisect.insort(buf, (t, z))
        if len(buf) > BOOT_SAMPLES:
            del buf[:len(buf) - BOOT_SAMPLES]
        return buf[-1][1]

    def _process(self, tr, t, anchor, raw, rx, fp, reporter):
        pos = self.anchors.get(anchor)
        if pos is None:
            return
        z = self.correct(anchor, raw)
        if z <= 0.05 or z > self._max_range(pos):
            # physically impossible here: the DW1000 occasionally reports
            # tens of metres (seen: 37-189 m in a 2.6 m room)
            tr.implausible += 1
            return
        zf = self._range_value(tr, anchor, t, z)
        tr.latest[anchor] = (t, z, raw)
        tr.sample_ts.append(t)
        if tr.first_t is None:
            tr.first_t = t
        link = tr.links.setdefault(anchor, {})
        link.update({"range": raw, "corrected": z, "filtered": zf, "rx": rx, "fp": fp,
                     "src": reporter, "t": t})

        if tr.f is None:
            self._bootstrap(tr, t)
            return

        p = self.params
        if t < tr.f.t - p["max_late_ms"]:
            tr.dropped_late += 1
            return

        tr.f.predict(t)

        ax, ay, az = pos
        tag_z = self.tag_z()
        sigma, bias, blockers = p["sigma_r"], 0.0, []
        if self.nlos_enabled and self.nlos_mod and self.scene.get("obstacles"):
            blockers = self.nlos_mod.los_blockers(
                (ax, ay, az), (tr.f.x[0], tr.f.x[1], tag_z), self.scene["obstacles"])
            sigma = self.nlos_mod.measurement_sigma(blockers, p["sigma_r"])
            bias = self.nlos_mod.nlos_bias(blockers)

        # still_long may drop this range only while the other anchors can still
        # fix the position: with a wrong anchor position (one range always
        # "too long") it would otherwise starve the validity gate
        others = sum(1 for a, ta in tr.acc_t.items() if a != anchor and t - ta <= p["fresh_ms"])
        tr.f.drop_ok = others >= p["min_anchors"]
        ok, y, nu, S = tr.f.update(ax, ay, az - tag_z, zf - bias, sigma,
                                   p["huber_k"], p["gate"], self.anchor_index.get(anchor),
                                   p["huber_k_long"])
        link.update({"residual": y, "nu": nu, "accepted": ok, "sigma": sigma,
                     "bias": bias, "blocked": bool(blockers),
                     "atten": max([ob.get("atten", 1.0) for ob in blockers], default=0.0),
                     "obstacles": [ob.get("id") for ob in blockers]})
        if ok:
            tr.last_accept = t
            tr.accepted_ts.append(t)
            tr.acc_t[anchor] = t
            tr.bias = tr.f.x[4]
            tr.bias_known = True
            if tr.f.n_off:
                for aid, i in self.anchor_index.items():
                    tr.offsets[aid] = tr.f.x[N_STATE + i]
            tr.warm += 1
        elif t - (tr.last_accept or t) > p["reinit_ms"]:
            # lost: every sample rejected for a while -> restart from geometry
            self._bootstrap(tr, t, force=True)
            return

        self._clamp_to_room(tr)

    def _clamp_to_room(self, tr):
        room = self.scene.get("room", {})
        w, d = room.get("width", 0), room.get("depth", 0)
        if w <= 0 or d <= 0:
            return
        m = self.params["room_margin"]
        x = tr.f.x
        cx = min(max(x[0], -m), w + m)
        cy = min(max(x[1], -m), d + m)
        if cx != x[0]:
            x[0], x[2] = cx, 0.0
        if cy != x[1]:
            x[1], x[3] = cy, 0.0

    def _reported(self, tr, px, py, vx, vy, age):
        """The position shown: the filter's, or with "smooth" the mean of its
        last smooth_window outputs. One entry per filter step: estimate()
        calls without a new sample in between add nothing."""
        p = self.params
        t = tr.f.t
        win = tr.post
        if not win or t > win[-1][0]:
            win.append((t, px, py))
        n = max(int(p["smooth_window"]), 1)
        while win and (len(win) > n or t - win[0][0] > p["window_ms"]):
            del win[0]
        if p["smooth"] < 0.5:
            return px, py, vx, vy
        return sum(q[1] for q in win) / len(win), sum(q[2] for q in win) / len(win), vx, vy

    # ----- output ------------------------------------------------------------
    def estimate(self, tag, now_ms=None):
        """Current estimate (now_ms: the time it is asked for, for its age).

        "valid" is False when the filter cannot fix a 2-D position (fewer than
        min_anchors anchors accepted within fresh_ms of the filter time, or
        sigma above valid_sigma) or when its newest data is older than
        fresh_ms; x, y are then the last valid position, held."""
        tr = self.tracks.get(tag)
        # a freshly (re)initialised track is not shown until it has absorbed
        # a couple of samples per anchor: the seed alone can be decimetres off
        if tr is None or tr.f is None or tr.warm < 2 * max(len(tr.latest), 2):
            return None
        p = self.params
        t_ref = tr.f.t if now_ms is None else max(now_ms, tr.f.t)
        tr.sample_ts = [t for t in tr.sample_ts if t_ref - t <= 1000]
        tr.accepted_ts = [t for t in tr.accepted_ts if t_ref - t <= 1000]
        px, py, vx, vy, bias = tr.f.x[:N_STATE]
        sigma = tr.f.sigma()
        n_fresh = sum(1 for ta in tr.acc_t.values() if tr.f.t - ta <= p["fresh_ms"])
        need = max(min(int(p["min_anchors"]), len(self.anchors)), 1)
        fixable = n_fresh >= need and sigma <= p["valid_sigma"]
        age = t_ref - tr.f.t
        if fixable:
            px, py, vx, vy = self._reported(tr, px, py, vx, vy, age)
            tr.hold = (px, py)
        else:
            tr.reset_output()
            if tr.hold is not None:
                px, py = tr.hold
            vx = vy = 0.0
        tag_z = self.tag_z()
        flat = []
        for anchor, (t, z, _raw) in tr.latest.items():
            pos = self.anchors.get(anchor)
            if pos and t_ref - t <= 2000:
                flat.append((pos[0], pos[1], horizontal_range(z, pos[2] - tag_z)))
        ok_geo, slack = geometry_check(flat)
        return {
            "id": tag, "t": tr.f.t,
            "x": px, "y": py, "z": tag_z, "vx": vx, "vy": vy,
            "range_bias": bias,
            "anchor_offsets": dict(tr.offsets),
            # an anchor whose own offset exceeds 20 cm: DW1000 modules differ by
            # a few cm, so the scene position of that anchor is probably wrong
            "offset_warning": sorted(a for a, v in tr.offsets.items() if abs(v) > 0.20),
            "sigma": sigma, "confidence": 1.0 / (1.0 + sigma),
            "valid": fixable and age <= p["fresh_ms"],
            "held": not fixable, "anchors_fresh": n_fresh, "age_ms": age,
            "still": self._still(tr),         # standing (zero-velocity mode)
            "calibrated": self.calibrated,    # offsets from the static calibration
            "ambiguous": False,
            "geometry_ok": ok_geo, "geometry_slack": slack,
            "used": len(tr.accepted_ts), "rate_hz": len(tr.sample_ts),
            "reinits": tr.reinits, "late": tr.dropped_late,
            "implausible": tr.implausible,
            "links": {a: dict(v) for a, v in tr.links.items()},
        }
