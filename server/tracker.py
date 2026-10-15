"""Tag tracking engine — measurement bookkeeping + EKF on slant ranges.

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

Everything here is plain Python (no numpy) so the server keeps its two
dependencies, and the same code runs in tools/simulate.py and
tools/evaluate.py for offline replay.
"""

import math

# ---------------------------------------------------------------------------
# tuning
# ---------------------------------------------------------------------------
DEFAULT_TRACKING = {
    # Positioning algorithm:
    #   "ekf"     position EKF on slant ranges (this project's tracker)
    #   "wls_rkf" Fan & Du 2022 (docs/papers/04): one Kalman filter per anchor
    #             range, chi-square NLOS test, weighted least squares position
    #   "ekf_nlos" the EKF, plus the per-anchor range filter and NLOS test of
    #             Fan & Du: a range judged NLOS gets the paper's weight
    #             (R scaled by gamma/chi2) and its filter follows the EKF
    "algorithm": "ekf",
    # White-acceleration process noise (m/s^2). A person walking or a pushed
    # trolley changes speed gently; 0.8 keeps straight walks straight while
    # still following a 90-degree turn without overshooting the corner.
    "sigma_a": 0.8,
    # LOS range noise of one DW1000 two-way-ranging sample (m).
    "sigma_r": 0.12,
    # Normalised innovation |y|/sqrt(S) where down-weighting starts (Huber).
    "huber_k": 2.5,
    # Same, for ranges LONGER than predicted. A body or wall in the path only
    # ever makes UWB read long, so those are more suspicious than short ones;
    # a smaller value down-weights them earlier. 0 = use huber_k.
    "huber_k_long": 0.0,
    # Normalised innovation beyond which a sample is rejected outright.
    "gate": 6.0,
    # A sample older than the filter state by more than this is dropped.
    "max_late_ms": 400,
    # No accepted sample for this long -> re-initialise from multilateration.
    "reinit_ms": 1500,
    # How far outside the room rectangle the estimate may go (m).
    "room_margin": 0.5,
    # Velocity correlation time (s), Singer / Ornstein-Uhlenbeck model: the
    # velocity decays as exp(-dt/tau) between samples. For the usual 40 ms
    # spacing this is a constant-velocity model; across a WiFi stall of a few
    # seconds it stops the estimate from coasting on a noisy velocity (seen:
    # 45 cm of drift for a tag lying still). 0 = pure constant velocity.
    "vel_tau_s": 1.0,
    # 1 = estimate the range offset shared by all anchors online (needs 3+).
    "estimate_bias": 1.0,
    # The reported position may be extrapolated along the velocity to "now"
    # by at most this much (ms). Off by default: in simulation it did not
    # lower the time-synchronised error but added sideways wobble.
    "extrapolate_ms": 0,
    # --- wls_rkf only ---
    # Process noise of each per-anchor range filter (m/s^2), paper: 0.5.
    "rkf_sigma_u": 0.5,
    # NLOS threshold on the squared Mahalanobis distance; chi2(1) at
    # alpha = 0.001 is 6.2 (paper). A range is NLOS only if it also reads
    # LONGER than predicted.
    "nlos_chi2": 6.2,
}

ALGORITHMS = ("ekf", "wls_rkf", "ekf_nlos")

# Anchor copies of a link are used only when the tag itself has sent nothing
# for this long (ms). The tag's WiFi uplink stalls for ~3 s now and then and
# delivers the backlog afterwards, so a short hold switched to the anchors'
# one-link-at-a-time batches exactly when they do the most damage.
SOURCE_HOLD_MS = 10000
# Fresh-sample window used for bootstrapping / re-initialisation (ms).
FIX_WINDOW_MS = 700
# wls_rkf: anchors heard within this window take part in the WLS solve; their
# weight shrinks with the prediction variance of their range filter (ms).
WLS_FRESH_MS = 3000


def tracking_params(scene):
    """Tracking parameters: defaults overridden by scene["tracking"]."""
    out = dict(DEFAULT_TRACKING)
    raw = (scene or {}).get("tracking") or {}
    for k in out:
        if k not in raw:
            continue
        if k == "algorithm":
            if str(raw[k]) in ALGORITHMS:
                out[k] = str(raw[k])
            continue
        try:
            out[k] = float(raw[k])
        except (TypeError, ValueError):
            pass
    return out


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

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
# Per-anchor differences remain for the static calibration to remove.
# ---------------------------------------------------------------------------

N_STATE = 5
BIAS_SIGMA0 = 0.3          # prior on b (m)
BIAS_WALK = 0.003          # random walk of b (m / sqrt(s))
BIAS_LIMIT = (-0.5, 1.0)   # physically plausible range of b (m)


class TagFilter:
    def __init__(self, px, py, t_ms, sigma_a, sigma_pos=0.5, bias=0.0,
                 estimate_bias=True, vel_tau=0.0):
        n = N_STATE
        self.x = [px, py, 0.0, 0.0, bias]
        self.P = [[0.0] * n for _ in range(n)]
        self.P[0][0] = self.P[1][1] = sigma_pos * sigma_pos
        self.P[2][2] = self.P[3][3] = 1.0
        self.P[4][4] = BIAS_SIGMA0 ** 2 if estimate_bias else 0.0
        self.estimate_bias = estimate_bias
        self.t = t_ms
        self.sigma_a = sigma_a
        self.vel_tau = vel_tau

    def predict(self, t_ms):
        dt = (t_ms - self.t) / 1000.0
        if dt <= 0:
            return
        self.t = t_ms
        dt = min(dt, 2.0)
        n, x, P = N_STATE, self.x, self.P
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
        q = self.sigma_a ** 2
        dt2 = dt * dt
        q11, q13, q33 = q * dt2 * dt2 / 4.0, q * dt2 * dt / 2.0, q * dt2
        FPF[0][0] += q11; FPF[1][1] += q11
        FPF[0][2] += q13; FPF[2][0] += q13
        FPF[1][3] += q13; FPF[3][1] += q13
        FPF[2][2] += q33; FPF[3][3] += q33
        if self.estimate_bias:
            FPF[4][4] += BIAS_WALK * BIAS_WALK * dt
        self.P = FPF
        # keep sigma interpretable during long outages
        for i, cap in ((0, 25.0), (1, 25.0), (2, 4.0), (3, 4.0)):
            if self.P[i][i] > cap:
                s = math.sqrt(cap / self.P[i][i])
                for j in range(n):
                    self.P[i][j] *= s
                    self.P[j][i] *= s

    def predicted_range(self, ax, ay, dz):
        dx, dy = self.x[0] - ax, self.x[1] - ay
        rho = math.sqrt(dx * dx + dy * dy + dz * dz) or 1e-6
        return rho, (dx / rho, dy / rho)

    def update(self, ax, ay, dz, z, sigma, huber_k, gate, huber_k_long=0.0):
        """One slant-range update. Returns (accepted, y, nu, S)."""
        n, P = N_STATE, self.P
        rho, (h0, h1) = self.predicted_range(ax, ay, dz)
        hb = 1.0 if self.estimate_bias else 0.0
        H = (h0, h1, 0.0, 0.0, hb)
        y = z - (rho + hb * self.x[4])
        PHt = [P[i][0] * h0 + P[i][1] * h1 + P[i][4] * hb for i in range(n)]
        HPHt = h0 * PHt[0] + h1 * PHt[1] + hb * PHt[4]
        R = sigma * sigma
        S = HPHt + R
        nu = y / math.sqrt(S)
        if abs(nu) > gate:
            return False, y, nu, S
        k = huber_k_long if (nu > 0 and huber_k_long > 0) else huber_k
        if abs(nu) > k:                       # Huber: inflate R, do not reject
            R *= abs(nu) / k
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
        return True, y, nu, S

    def sigma(self):
        return math.sqrt(max(self.P[0][0], 0.0) + max(self.P[1][1], 0.0))


# ---------------------------------------------------------------------------
# WLS-RKF  (Fan & Du 2022, "NLOS Error Mitigation Using Weighted Least
# Squares and Kalman Filter in UWB Positioning", arXiv:2205.05939 —
# docs/papers/04_Fan_2022_WLS_RKF_NLOS_Kalman.pdf, Algorithm 1)
#
#   1. one Kalman filter KF_i per anchor on the range, state [r, r_dot],
#      constant velocity, process noise on r_dot (paper eq. 3-4)
#   2. NLOS test (eq. 13-15): gamma = (r - d)^2 / (H P H^T + R) with
#      d = predicted range; NLOS if gamma > chi2(1, 0.001) = 6.2 AND r > d
#   3. LOS: update KF_i with r, use the filtered range, weight 1
#      NLOS: use the predicted range d, weight sqrt(6.2 / gamma)  (eq. 16-17)
#   4. position = weighted least squares on the ranges (eq. 18)
#   5. NLOS filters are then updated with the distance from the WLS
#      position to their anchor (eq. 19), so they follow the LOS solution
#
# Adapted to this system (and documented as such in PERBAIKAN_AKURASI.md):
#   * ranges are 3-D slant ranges: rho_i = sqrt(dx^2 + dy^2 + dz^2)
#   * the paper's ranges were calibrated; ours carry a common offset (~0.75 m
#     uncalibrated antenna delay), so the WLS solves [x, y, b] with a
#     random-walk prior on b (same role as the EKF's b state)
#   * samples arrive one anchor at a time, not in synchronous epochs: every
#     sample triggers a solve, and each anchor's range is propagated to the
#     solve time with its own filter (the per-anchor filters make this easy)
# ---------------------------------------------------------------------------

class RangeKF:
    """Kalman filter on one anchor's range, state [r, r_dot]."""

    def __init__(self, r, t_ms, sigma_r):
        self.r, self.v = r, 0.0
        self.P = [[sigma_r * sigma_r, 0.0], [0.0, 1.0]]
        self.t = t_ms

    def predict(self, t_ms, sigma_u):
        dt = min(max((t_ms - self.t) / 1000.0, 0.0), 2.0)
        if dt <= 0:
            return
        self.t = t_ms
        self.r += self.v * dt
        P = self.P
        # P = A P A^T + Gamma Q Gamma^T, A = [[1, dt], [0, 1]], Gamma = [0, dt]
        p00 = P[0][0] + 2 * dt * P[0][1] + dt * dt * P[1][1]
        p01 = P[0][1] + dt * P[1][1]
        p11 = P[1][1] + (sigma_u * dt) ** 2
        self.P = [[p00, p01], [p01, p11]]

    def range_at(self, t_ms):
        """Range propagated to t_ms without touching the state."""
        dt = min(max((t_ms - self.t) / 1000.0, 0.0), 2.0)
        return self.r + self.v * dt

    def var_at(self, t_ms, sigma_u):
        """Variance of range_at(t_ms): grows while the anchor is not heard."""
        dt = min(max((t_ms - self.t) / 1000.0, 0.0), 2.0)
        P = self.P
        return P[0][0] + 2 * dt * P[0][1] + dt * dt * P[1][1] + (sigma_u * dt * dt) ** 2

    def gamma(self, z, R):
        return (z - self.r) ** 2 / (self.P[0][0] + R)

    def update(self, z, R):
        P = self.P
        S = P[0][0] + R
        k0, k1 = P[0][0] / S, P[1][0] / S
        y = z - self.r
        self.r += k0 * y
        self.v += k1 * y
        self.P = [[(1 - k0) * P[0][0], (1 - k0) * P[0][1]],
                  [P[1][0] - k1 * P[0][0], P[1][1] - k1 * P[0][1]]]


class WlsRkfFilter:
    """Position from per-anchor range filters + NLOS-weighted least squares.

    Exposes the same surface the Engine reads from TagFilter (x, t, sigma(),
    sigma_a, vel_tau) so the output path, room clamp and UI are shared.
    """

    def __init__(self, px, py, t_ms, bias=0.0, estimate_bias=True):
        self.x = [px, py, 0.0, 0.0, bias]     # px, py, vx, vy, b
        self.t = t_ms
        self.kf = {}                          # anchor -> RangeKF
        self.sol = {}                         # anchor -> (r_hat, weight, nlos)
        self.Pb = BIAS_SIGMA0 ** 2 if estimate_bias else 0.0
        self.estimate_bias = estimate_bias
        self.Pxy = [[0.25, 0.0], [0.0, 0.25]]
        self.sigma_a = 0.0                    # unused, kept for configure()
        self.vel_tau = 0.0

    def sigma(self):
        return math.sqrt(max(self.Pxy[0][0], 0.0) + max(self.Pxy[1][1], 0.0))

    def step(self, anchor, z, t_ms, anchors, tag_z, p, fresh_ms):
        """One range sample z (calibrated, still containing the common
        offset) from `anchor`. Returns (los, innovation, nu, weight)."""
        R = p["sigma_r"] ** 2
        kf = self.kf.get(anchor)
        new = kf is None
        if new:                                   # first sample seeds the filter
            kf = self.kf[anchor] = RangeKF(z, t_ms, p["sigma_r"])
            gamma, y = 0.0, 0.0
        else:
            kf.predict(t_ms, p["rkf_sigma_u"])
            y = z - kf.r
            gamma = kf.gamma(z, R)
        chi2 = p["nlos_chi2"]
        nlos = (not new) and gamma > chi2 and y > 0     # one-sided test (eq. 15)
        if nlos:
            w = math.sqrt(chi2 / gamma)                 # eq. 17
        else:
            if not new:
                kf.update(z, R)
            w = 1.0
        self.sol[anchor] = (w, nlos)

        # Weighted least squares on every anchor heard in the last few seconds,
        # each range propagated to now by its own filter. The paper's epochs
        # are synchronous (all anchors every 50 ms); here an anchor may be
        # silent for a second or two, so its weight also carries the growing
        # prediction variance: W = w^2 / (R + var). Using only the anchors of
        # the last 0.7 s left 2-3 rows and the solution jumped by up to 2 m.
        rows = []
        for a, (wa, _nl) in self.sol.items():
            f = self.kf.get(a)
            pos = anchors.get(a)
            if f is None or pos is None or t_ms - f.t > fresh_ms:
                continue
            W = wa * wa / (R + f.var_at(t_ms, p["rkf_sigma_u"]))
            rows.append((pos, f.range_at(t_ms), W))
        self._solve(rows, t_ms, tag_z, p)

        # NLOS: the filter follows the LOS solution instead of the biased range
        if nlos:
            pos = anchors.get(anchor)
            if pos is not None:
                rho = math.sqrt((self.x[0] - pos[0]) ** 2 + (self.x[1] - pos[1]) ** 2
                                + (tag_z - pos[2]) ** 2)
                kf.update(rho + self.x[4], R)
        nu = math.copysign(math.sqrt(gamma), y) if gamma > 0 else 0.0
        return (not nlos), y, nu, w

    def _solve(self, rows, t_ms, tag_z, p):
        """rows: (anchor_xyz, range, weight W = 1/variance). Solves [x, y]
        and, with 4+ anchors, the common offset b (else b is held)."""
        dt = max((t_ms - self.t) / 1000.0, 0.0)
        # 3 ranges for x, y; a 4th before b is free too: with exactly as many
        # ranges as unknowns every bit of noise lands in the position
        if len(rows) < 3:
            return
        use_b = self.estimate_bias and len(rows) >= 4
        if use_b:
            self.Pb = min(self.Pb + BIAS_WALK * BIAS_WALK * dt, BIAS_SIGMA0 ** 2)
        x, y, b = self.x[0], self.x[1], self.x[4]
        b_prior = b
        n = 3 if use_b else 2
        cov = None
        for _ in range(6):
            A = [[0.0] * n for _ in range(n)]
            g = [0.0] * n
            for (ax, ay, az), r, W in rows:
                dx, dy, dz = x - ax, y - ay, tag_z - az
                rho = math.sqrt(dx * dx + dy * dy + dz * dz) or 1e-6
                J = (dx / rho, dy / rho, 1.0)[:n]
                res = r - (rho + b)
                for i in range(n):
                    g[i] += J[i] * W * res
                    for j in range(n):
                        A[i][j] += J[i] * W * J[j]
            if use_b and self.Pb > 0:            # random-walk prior on b
                A[2][2] += 1.0 / self.Pb
                g[2] += (b_prior - b) / self.Pb
            cov = _invert(A)
            if cov is None:
                return
            step = [sum(cov[i][j] * g[j] for j in range(n)) for i in range(n)]
            x += step[0]
            y += step[1]
            if use_b:
                b += step[2]
            if abs(step[0]) + abs(step[1]) < 1e-4:
                break
        lo, hi = BIAS_LIMIT
        b = min(max(b, lo), hi)
        # velocity from the position change, lightly smoothed (output only)
        if dt > 0:
            a = min(dt / 0.5, 1.0)
            self.x[2] += a * ((x - self.x[0]) / dt - self.x[2])
            self.x[3] += a * ((y - self.x[1]) / dt - self.x[3])
        self.x[0], self.x[1], self.x[4] = x, y, b
        self.Pxy = [[cov[0][0], cov[0][1]], [cov[1][0], cov[1][1]]]
        if use_b:
            self.Pb = cov[2][2]
        self.t = max(self.t, t_ms)


def _invert(A):
    """Inverse of a small symmetric matrix (2x2 or 3x3), None if singular."""
    n = len(A)
    M = [list(A[i]) + [1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[piv][c]) < 1e-12:
            return None
        M[c], M[piv] = M[piv], M[c]
        f = M[c][c]
        M[c] = [v / f for v in M[c]]
        for r in range(n):
            if r != c and M[r][c] != 0.0:
                k = M[r][c]
                M[r] = [vr - k * vc for vr, vc in zip(M[r], M[c])]
    return [row[n:] for row in M]


# ---------------------------------------------------------------------------
# per-tag track
# ---------------------------------------------------------------------------

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
        self.implausible = 0          # samples dropped as physically impossible
        self.first_t = None           # first sample seen (bootstrap patience)
        self.warm = 0                 # accepted samples since the last (re)init
        self.rkf = {}                 # ekf_nlos: anchor -> RangeKF (NLOS test)


class Engine:
    """Owns every tag track. Not thread-safe: the caller holds its lock."""

    def __init__(self):
        self.scene = {"room": {"width": 5.0, "depth": 4.0}, "anchors": [],
                      "obstacles": [], "tag_z": 1.3}
        self.nlos_enabled = True
        self.anchors = {}             # id -> (x, y, z)
        self.cal = {}                 # id -> (bias, scale)
        self.params = dict(DEFAULT_TRACKING)
        self.tracks = {}              # tag -> TagTrack
        self.clocks = {}              # reporting device -> ClockSync
        self.last_dev_ts = {}         # (reporter, anchor, tag) -> device ts consumed
        self.tag_reported = {}        # (anchor, tag) -> server ms the tag last reported it
        self.on_sample = None         # optional callbacks (recorder)
        self.nlos_mod = None          # scene module (obstacle tests), injected

    # ----- configuration ---------------------------------------------------
    def configure(self, scene, nlos_enabled=True, extra_anchors=None, nlos_mod=None):
        """(Re)load geometry + calibration + tuning. Tracks are kept unless the
        caller also calls reset() (it should, when anchors move)."""
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
        old_algo = self.params.get("algorithm")
        self.params = tracking_params(scene)
        if self.params["algorithm"] != old_algo:
            # a different algorithm needs a fresh track (other filter type)
            for tr in self.tracks.values():
                tr.f = None
                tr.first_t = None
        for tr in self.tracks.values():
            if tr.f is not None:
                tr.f.sigma_a = self.params["sigma_a"]
                tr.f.vel_tau = self.params["vel_tau_s"]

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
        out = []
        for anchor, (t, z, _raw) in tr.latest.items():
            pos = self.anchors.get(anchor)
            if pos and now - t <= FIX_WINDOW_MS:
                out.append((pos[0], pos[1], pos[2], z))
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
        # the common offset learned so far still applies to the new fix
        fixes = [(ax, ay, az, r - tr.bias) for ax, ay, az, r in fixes]
        sol = multilaterate(fixes, self.tag_z(), room.get("width", 0), room.get("depth", 0))
        if sol is None:
            return False
        # With 3+ anchors an inconsistent set (NLOS) would seed a wrong track;
        # wait for a better one unless we have been lost for a while.
        if len(fixes) >= 3 and sol["rms"] > 0.6 and not force:
            return False
        tr.warm = 0
        est_b = self.params["estimate_bias"] >= 0.5 and len(self.anchors) >= 3
        if self.params["algorithm"] == "wls_rkf":
            tr.f = WlsRkfFilter(sol["x"], sol["y"], now, bias=tr.bias, estimate_bias=est_b)
        else:
            tr.f = TagFilter(sol["x"], sol["y"], now, self.params["sigma_a"], bias=tr.bias,
                             estimate_bias=est_b, vel_tau=self.params["vel_tau_s"])
        tr.last_accept = now
        tr.reinits += 1 if force else 0
        return True

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
        tr.latest[anchor] = (t, z, raw)
        tr.sample_ts.append(t)
        if tr.first_t is None:
            tr.first_t = t
        link = tr.links.setdefault(anchor, {})
        link.update({"range": raw, "corrected": z, "rx": rx, "fp": fp,
                     "src": reporter, "t": t})

        if tr.f is None:
            self._bootstrap(tr, t)
            return

        p = self.params
        if t < tr.f.t - p["max_late_ms"]:
            tr.dropped_late += 1
            return

        if isinstance(tr.f, WlsRkfFilter):
            self._process_wls(tr, t, anchor, z, link)
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

        # ekf_nlos: Fan & Du's NLOS identification (eq. 13-15) on this anchor's
        # own range history; an NLOS range keeps a reduced weight in the EKF
        # (their eq. 17 weight w = sqrt(chi2/gamma), i.e. R / w^2)
        nlos = False
        if p["algorithm"] == "ekf_nlos":
            R = p["sigma_r"] ** 2
            kf = tr.rkf.get(anchor)
            if kf is None:
                tr.rkf[anchor] = RangeKF(z, t, p["sigma_r"])
            else:
                kf.predict(t, p["rkf_sigma_u"])
                g = kf.gamma(z, R)
                nlos = g > p["nlos_chi2"] and z > kf.r
                if nlos:
                    sigma *= math.sqrt(g / p["nlos_chi2"])
                else:
                    kf.update(z, R)

        ok, y, nu, S = tr.f.update(ax, ay, az - tag_z, z - bias, sigma,
                                   p["huber_k"], p["gate"], p["huber_k_long"])
        if nlos:
            # the anchor's range filter follows the EKF instead of the biased
            # measurement (their eq. 19), so the NLOS episode stays detected
            rho, _h = tr.f.predicted_range(ax, ay, az - tag_z)
            tr.rkf[anchor].update(rho + tr.f.x[4], p["sigma_r"] ** 2)
        link.update({"residual": y, "nu": nu, "accepted": ok, "sigma": sigma,
                     "nlos": nlos, "bias": bias, "blocked": bool(blockers),
                     "atten": max([ob.get("atten", 1.0) for ob in blockers], default=0.0),
                     "obstacles": [ob.get("id") for ob in blockers]})
        if ok:
            tr.last_accept = t
            tr.accepted_ts.append(t)
            tr.bias = tr.f.x[4]
            tr.warm += 1
        elif t - (tr.last_accept or t) > p["reinit_ms"]:
            # lost: every sample rejected for a while -> restart from geometry
            self._bootstrap(tr, t, force=True)
            return

        self._clamp_to_room(tr)

    def _process_wls(self, tr, t, anchor, z, link):
        """WLS-RKF path of _process (algorithm = "wls_rkf")."""
        los, y, nu, w = tr.f.step(anchor, z, t, self.anchors, self.tag_z(),
                                  self.params, WLS_FRESH_MS)
        link.update({"residual": y, "nu": nu, "accepted": los, "nlos": not los,
                     "weight": w, "sigma": self.params["sigma_r"], "bias": 0.0,
                     "blocked": False, "atten": 0.0, "obstacles": []})
        tr.last_accept = t if los else tr.last_accept
        if los:
            tr.accepted_ts.append(t)
        tr.bias = tr.f.x[4]
        tr.warm += 1
        if t - (tr.last_accept or t) > self.params["reinit_ms"]:
            # every anchor judged NLOS for a while: the filters have locked
            # onto a wrong solution -> restart from geometry
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

    # ----- output ------------------------------------------------------------
    def estimate(self, tag, now_ms=None):
        """Current estimate. With now_ms the position is extrapolated along the
        velocity to that instant (at most extrapolate_ms), so the displayed
        point does not trail the tag by the uplink batching delay."""
        tr = self.tracks.get(tag)
        # a freshly (re)initialised track is not shown until it has absorbed
        # a couple of samples per anchor: the seed alone can be decimetres off
        if tr is None or tr.f is None or tr.warm < 2 * max(len(tr.latest), 2):
            return None
        t_ref = tr.f.t if now_ms is None else max(now_ms, tr.f.t)
        tr.sample_ts = [t for t in tr.sample_ts if t_ref - t <= 1000]
        tr.accepted_ts = [t for t in tr.accepted_ts if t_ref - t <= 1000]
        px, py, vx, vy, bias = tr.f.x
        ahead = min(t_ref - tr.f.t, self.params["extrapolate_ms"]) / 1000.0
        px += vx * ahead
        py += vy * ahead
        sigma = tr.f.sigma()
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
            "sigma": sigma, "confidence": 1.0 / (1.0 + sigma),
            "ambiguous": False,
            "geometry_ok": ok_geo, "geometry_slack": slack,
            "used": len(tr.accepted_ts), "rate_hz": len(tr.sample_ts),
            "reinits": tr.reinits, "late": tr.dropped_late,
            "implausible": tr.implausible,
            "links": {a: dict(v) for a, v in tr.links.items()},
        }
