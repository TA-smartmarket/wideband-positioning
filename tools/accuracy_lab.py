"""Accuracy research: can an EKF-based pipeline reach 2-4 cm? (simulation only)

    python tools/accuracy_lab.py              # numbers -> docs/hasil-simulasi/riset_akurasi.json
    python tools/accuracy_report.py           # figures + workbook from them

Nothing here changes the server: the tracker (server/tracker.py, algorithm
"G") is imported and extended in this file only, and every variant runs on
identical simulated data.

Why G stops at ~15 cm: at a stop its error is a BIAS, not noise. The pusher's
body lengthens the range to the anchors behind the trolley by 10-35 cm.
Averaging cannot remove a bias, and G's own offsets (b, d_i) half absorb it.

Result: BZ-EKF = G + K + F + B + hd + Z (the row "K+F+B+hd+Z"):
  K   static calibration: each anchor's range offset measured once with the
      tag at known points (the dashboard's calibration), error ~2 cm
  F   ... and trusted: the EKF no longer re-learns the offsets, so they cannot
      soak up the body shadow
  B   one more EKF state beta, the body shadow: the range a body adds to the
      anchors in the rear cone of the travel direction,
          h_i = rho_i + b + d_i + s_i * beta,   s_i in [0, 1]
      (cf. De Cock et al. 2023, Tian et al. 2019: a range-error model chosen by
      the body-tag-anchor orientation; there the orientation comes from an IMU,
      here from the UWB track itself)
  hd  the travel direction for s_i: a line fit through the estimates of the
      last 4 s of travel (>= 1 m), frozen at a stop
  Z   zero-velocity mode (ZUPT, Foxlin 2005; a two-model stop/move switch in
      the spirit of IMM, Blom & Bar-Shalom 1988): at a stop the velocity is
      pinned to 0 and the process noise made small, so a bad round of ranges
      no longer moves the estimate

  imu instead of hd: an IMU on the tag (Imu below). Its gyro gives the heading
      (true facing + 5 deg fixed + 3 deg random error): the track cannot show
      the direction of a leg shorter than ~1 m after a corner
      (tools/walk_test.py: 14-18 cm at such stops), a gyro can, as in De Cock
      2023 / Tian 2019. Its accelerometer switches the zero-velocity mode: a
      push is felt at once, while the ranges show it only after ~1 s at
      2.5 Hz (the error peaks of 50-60 cm when the trolley sets off)

Also tried (kept for comparison, mode tokens joined by "+"):
  S      stop solve: median of every anchor's ranges since the stop, minus the
         offsets, least squares for x, y
  FDE    fault detection & exclusion (RAIM, as in GNSS) on the stop solve: the
         anchor whose removal makes the fit consistent AND whose range is too
         LONG (NLOS only lengthens) is dropped
  RW     residual weighting (Chen 1999) of every subset of 3+ anchors
  HP     heading prior: anchors behind the travel direction get 1/4 weight
  W      a longer averaging window at a stop
  sym    symmetric Huber (beta explains the long ranges)
  HT     heading test at a stop: the heading whose shadow pattern fits the
         median ranges best (24 candidates); with 2 spare equations it picks
         wrong ones too often (walking test 7.5 -> 9.3 cm)
  HEAD_SHORT  a second, 0.4 m scale for the heading after corners: better on
         the shopping routes (7.5 -> 6.0 cm), worse on the rectangle
         (4.4 -> 5.0 cm), so off

Scenarios (radio model of tools/simulate.py, 9.5 x 5.9 m room, hardware
ranging rate, two laps with 6 s stops, 4 or 6 anchors):
  market       body shadowing +15 cm on anchors behind the pusher
  market-los   no body shadowing, no outliers (the floor of the noise model)
  robust       a different world per run, to check nothing is tuned to the
               model: body shadowing 8-35 cm over a random cone, plus random
               shelf NLOS episodes on every anchor (+15..60 cm, 10% of the time)
"""
import argparse
import itertools
import json
import math
import random
import sys
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import simulate as sim  # noqa: E402
import tracker_lab  # noqa: E402
import smoothing_study as st  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
tracker = sim.tracker
LAPS = 2
WALK = st.MARKET_WALK + st.MARKET_WALK[1:] * (LAPS - 1)
# the same room with two more anchors halfway along the long walls
MARKET6 = json.loads(json.dumps(st.MARKET))
MARKET6["anchors"] += [{"id": "anchor-6", "x": 4.75, "y": 0.0, "z": 1.74},
                       {"id": "anchor-7", "x": 4.75, "y": 5.9, "z": 1.74}]


def scene_for(n_anchors):
    return MARKET6 if n_anchors == 6 else st.MARKET


# ---------------------------------------------------------------------------
# radio model with configurable (and random) NLOS
# ---------------------------------------------------------------------------

def world(name, rng):
    if name == "market":
        return dict(body=0.15, cos=-0.35, outlier=0.04, shelf_p=0.0)
    if name == "market-los":
        return dict(body=0.0, cos=-0.35, outlier=0.0, shelf_p=0.0)
    if name == "robust":
        return dict(body=rng.uniform(0.08, 0.35), cos=rng.uniform(-0.6, -0.1), outlier=0.04,
                    shelf_p=0.10)
    raise ValueError(name)


def radio(scene, segs, rng, duration, w):
    """Like simulate.simulate_radio, with the NLOS model of world w."""
    anchors = scene["anchors"]
    tag_z = scene["tag_z"]
    bias = {a["id"]: sim.BIAS_COMMON + rng.gauss(0, sim.BIAS_SPREAD) for a in anchors}
    shelf = {a["id"]: 0.0 for a in anchors}            # current shelf NLOS excess
    dwell = 3.0
    p_on = w["shelf_p"] / max(1.0 - w["shelf_p"], 1e-6) / dwell   # per second
    poll = st.HW_POLL_MS
    out = []
    t = rng.uniform(0, poll)
    while t < duration * 1000:
        dt = poll / 1000.0
        for a in anchors:                               # two-state Markov chain
            if shelf[a["id"]] == 0.0:
                if rng.random() < p_on * dt:
                    shelf[a["id"]] = rng.uniform(0.15, 0.60)
            elif rng.random() < dt / dwell:
                shelf[a["id"]] = 0.0
        for i, a in enumerate(anchors):
            if rng.random() < sim.LOSS_P:
                continue
            tm = t + 7 * (2 * i + 1) + 3
            px, py = sim.truth_at(segs, tm / 1000)
            hx, hy = sim.heading_at(segs, tm / 1000)
            d = math.sqrt((px - a["x"]) ** 2 + (py - a["y"]) ** 2 + (tag_z - a["z"]) ** 2)
            r = d + bias[a["id"]] + rng.gauss(0, sim.NOISE_M)
            ux, uy = a["x"] - px, a["y"] - py
            un = math.hypot(ux, uy) or 1.0
            if w["body"] > 0 and (ux * hx + uy * hy) / un < w["cos"]:
                r += w["body"] + abs(rng.gauss(0, sim.BODY_NOISE))
            if rng.random() < w["outlier"]:
                r += rng.uniform(*sim.OUTLIER_RANGE)
            if shelf[a["id"]] > 0:
                r += shelf[a["id"]] + abs(rng.gauss(0, 0.03))
            out.append({"t": tm, "anchor": a["id"], "r": r,
                        "tag_rx": rng.random() >= sim.REPORT_LOSS_P})
        t += poll + rng.uniform(0, 10)
    return out, bias


# ---------------------------------------------------------------------------
# body-shadow EKF (research variant of tracker.TagFilter)
# ---------------------------------------------------------------------------

SHADE_C0, SHADE_CW = 0.2, 0.3      # rear cone: 0 at cos = -0.2, full at -0.5
BETA_SIGMA0 = 0.15                 # prior on beta (m)
BETA_LIMIT = (0.0, 0.6)
BETA_WALK = 0.005                  # random walk of beta (m / sqrt(s)): pushers differ


CAL_SIGMA = 0.02                   # error of a static calibration per anchor (m)
IMU_BIAS_DEG, IMU_NOISE_DEG = 5.0, 3.0   # gyro heading: fixed error per run + noise


IMU_START_MS, IMU_STILL_MS = 100, 300   # accelerometer: push seen after 0.1 s, rest after 0.3 s


class Imu:
    """What an IMU on the tag would tell the filter: the gyro heading (the
    true facing of the trolley, i.e. its last travel direction, turned by a
    fixed error plus noise) and whether the accelerometer feels the trolley
    standing (a push is felt 0.1 s after it starts, rest 0.3 s after it)."""

    def __init__(self, segs, seed):
        self.segs = segs
        self.rng = random.Random(seed * 31 + 7)
        self.off = math.radians(self.rng.gauss(0, IMU_BIAS_DEG))

    def heading(self, t_ms):
        hx, hy = sim.heading_at(self.segs, t_ms / 1000)
        a = math.atan2(hy, hx) + self.off + math.radians(self.rng.gauss(0, IMU_NOISE_DEG))
        return math.cos(a), math.sin(a)

    def still(self, t_ms):
        mv, since = st.moving_at(self.segs, (t_ms - IMU_START_MS) / 1000)
        return not mv and since * 1000 >= IMU_STILL_MS - IMU_START_MS


def imu_model(segs, seed):
    return Imu(segs, seed)
# zero-velocity mode (ZUPT, as in foot-mounted INS; a two-model stop/move
# switch in the spirit of IMM): once the trolley has been slow for a while the
# filter assumes it stands: velocity pinned to 0, small process noise, so one
# bad round of ranges can no longer move the estimate
ZUPT_SLOW, ZUPT_AFTER_MS = 0.20, 400     # enter: speed < 0.2 m/s for 0.4 s
ZUPT_SIGMA_A, ZUPT_SIGMA_V = 0.05, 0.02  # while still (m/s^2, m/s)
ZUPT_EXIT_NU, ZUPT_EXIT_N = 1.5, 3       # leave: 3 of the last 4 |nu| > 1.5


class BodyFilter(tracker.TagFilter):
    """TagFilter with one more state beta (last in x): the range a body behind
    the trolley adds to the anchors in the rear cone of the travel direction.

    Options (class attributes, set per run):
      USE_BETA    add the beta state (the engine then has one extra "offset")
      HEAD_DISP   heading = line fit through the estimates of the last 4 s of
                  travel (spanning >= 1 m) instead of the EKF velocity: does
                  not swing while braking or when one outlier makes the
                  estimate jump
      CALIBRATED  per-anchor offsets come from a static calibration: b is not
                  estimated and d_i only within the calibration error
      ZUPT        zero-velocity mode at stops (see ZUPT_* above)
      FREEZE      with CALIBRATED: d_i are not learned at all (the
                  calibration is trusted; only beta adapts)"""
    USE_BETA = True
    HEAD_DISP = False
    ZUPT = False
    FREEZE = False
    STILL_LONG = None                # standing: drop ranges longer by > this many sigma
    STILL_RESET = 0.0                # standing: position variance added on stopping (m^2)
    IMU = None                       # t_ms -> heading from a gyro on the tag
    IMU_STILL = None                 # t_ms -> standing, from its accelerometer
    HEAD_DIST, HEAD_TIME, HEAD_SPEED = 1.0, 4000, 0.25
    HEAD_SHORT, HEAD_TURN = 0.0, 50.0    # second, short scale: tried, off (see doc)
    CALIBRATED = False

    def __init__(self, *a, **k):
        if self.CALIBRATED:
            k["estimate_bias"] = False
        super().__init__(*a, **k)
        self.ib = len(self.x) - 1 if (self.n_off and self.USE_BETA) else None
        if self.ib is not None:
            self.x[self.ib] = min(max(self.x[self.ib], 0.0), BETA_LIMIT[1])
            self.P[self.ib][self.ib] = BETA_SIGMA0 ** 2
        if self.CALIBRATED:
            for i in range(tracker.N_STATE, len(self.x)):
                if i != self.ib:
                    self.P[i][i] = 0.0 if self.FREEZE else CAL_SIGMA ** 2
        self.heading = None
        self.trail = []
        self.born = self.t
        self.still = False
        self.slow_since = None
        self.recent = []
        self.sigma_a_move = self.sigma_a

    def predict(self, t_ms):
        dt = min(max((t_ms - self.t) / 1000.0, 0.0), 2.0)
        if self.ZUPT:
            self.sigma_a = ZUPT_SIGMA_A if self.still else self.sigma_a_move
        super().predict(t_ms)
        if self.ib is not None and dt > 0:
            self.P[self.ib][self.ib] += (BETA_WALK ** 2 - tracker.ANCHOR_OFF_WALK ** 2) * dt
        if self.FREEZE:
            n = len(self.x)
            for i in range(tracker.N_STATE, n):
                if i != self.ib:
                    for q in range(n):
                        self.P[i][q] = self.P[q][i] = 0.0
        if self.ZUPT and self.still and dt > 0:
            self._linear(2, 0.0, ZUPT_SIGMA_V ** 2)
            self._linear(3, 0.0, ZUPT_SIGMA_V ** 2)

    def _heading_fit(self):
        """Heading = principal direction of the estimated positions over the
        newest HEAD_DIST metres of travel (within HEAD_TIME ms). A line
        through ~10 points is far steadier than the difference of two (the
        track wanders 10-30 cm across while moving); jitter or one jump at a
        stop never spans it, so a stop keeps its approach direction.
        Two scales: the long stretch normally; the short one (HEAD_SHORT)
        when it turned away from the long one by > HEAD_TURN degrees, i.e.
        round a corner, where the long stretch still holds the old aisle."""
        x = self.x
        if self.t - self.born < 3000 or (self.ZUPT and self.still):
            self.trail = []                  # converging / standing: no travel
            return
        self.trail = [p for p in self.trail if self.t - p[0] <= self.HEAD_TIME] + [(self.t, x[0], x[1])]
        if math.hypot(x[2], x[3]) < self.HEAD_SPEED:
            return
        h_long = self._line(self.HEAD_DIST, newest=False)
        h_short = self._line(self.HEAD_SHORT) if self.HEAD_SHORT else None
        ref = h_long or self.heading
        if h_short and ref and h_short[0] * ref[0] + h_short[1] * ref[1] < math.cos(math.radians(self.HEAD_TURN)):
            self.heading = h_short           # a corner: forget the old aisle
            keep = self.trail[-1]
            self.trail = [p for p in self.trail
                          if math.hypot(keep[1] - p[1], keep[2] - p[2]) <= self.HEAD_SHORT]
        elif h_long:
            self.heading = h_long

    def _line(self, dist, newest=True):
        """Direction of a line fit through the trail (the whole HEAD_TIME of
        it, or only the newest points spanning dist), None while it does not
        span dist metres. More points = steadier on a long aisle."""
        tr = self.trail
        if newest:
            i = len(tr) - 1
            while i > 0 and math.hypot(tr[-1][1] - tr[i][1], tr[-1][2] - tr[i][2]) < dist:
                i -= 1
            tr = tr[i:]
        if len(tr) < 5 or math.hypot(tr[-1][1] - tr[0][1], tr[-1][2] - tr[0][2]) < dist:
            return None
        mx = sum(p[1] for p in tr) / len(tr)
        my = sum(p[2] for p in tr) / len(tr)
        sxx = sum((p[1] - mx) ** 2 for p in tr)
        syy = sum((p[2] - my) ** 2 for p in tr)
        sxy = sum((p[1] - mx) * (p[2] - my) for p in tr)
        ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
        hx, hy = math.cos(ang), math.sin(ang)
        if hx * (tr[-1][1] - tr[0][1]) + hy * (tr[-1][2] - tr[0][2]) < 0:
            hx, hy = -hx, -hy                # point the way the trolley went
        return hx, hy

    def _linear(self, j, z, R):
        """Scalar update of state j with a direct measurement z (variance R)."""
        x, P = self.x, self.P
        n = len(x)
        S = P[j][j] + R
        K = [P[i][j] / S for i in range(n)]
        y = z - x[j]
        for i in range(n):
            x[i] += K[i] * y
        self.P = [[P[i][q] - K[i] * P[j][q] for q in range(n)] for i in range(n)]

    def _zupt_mode(self, nu):
        """Enter / leave the zero-velocity mode after each range update."""
        if not self.ZUPT:
            return
        if self.IMU_STILL:                   # the accelerometer knows
            was, self.still = self.still, self.IMU_STILL(self.t)
            if was and not self.still:
                self.P[2][2] += 0.36
                self.P[3][3] += 0.36
                self.P[0][0] += 0.01
                self.P[1][1] += 0.01
            return
        if self.still:
            self.recent = (self.recent + [abs(nu) > ZUPT_EXIT_NU])[-4:]
            if sum(self.recent) >= ZUPT_EXIT_N:          # ranges keep moving: go
                self.still, self.slow_since, self.recent = False, None, []
                self.P[2][2] += 0.36
                self.P[3][3] += 0.36
                self.P[0][0] += 0.01
                self.P[1][1] += 0.01
            return
        if math.hypot(self.x[2], self.x[3]) < ZUPT_SLOW:
            if self.slow_since is None:
                self.slow_since = self.t
            elif self.t - self.slow_since >= ZUPT_AFTER_MS:
                self.still, self.recent = True, []
                if self.STILL_RESET:         # settle afresh from the ranges at the stop
                    self.P[0][0] += self.STILL_RESET
                    self.P[1][1] += self.STILL_RESET
        else:
            self.slow_since = None

    def shade(self, ax, ay, h=None, at=None):
        """0..1: how far anchor (ax, ay) lies in the rear cone of heading h
        (default: the filter's) seen from at (default: the filter's position)."""
        h = h or self.heading
        if self.ib is None or h is None:
            return 0.0
        px, py = at or (self.x[0], self.x[1])
        ux, uy = ax - px, ay - py
        un = math.hypot(ux, uy) or 1.0
        c = (ux * h[0] + uy * h[1]) / un
        return min(max((-c - SHADE_C0) / SHADE_CW, 0.0), 1.0)

    def update(self, ax, ay, dz, z, sigma, huber_k, gate, k=None, huber_k_long=0.0):
        x, P = self.x, self.P
        n = len(x)
        if self.IMU:
            self.heading = self.IMU(self.t)
        elif self.HEAD_DISP:
            self._heading_fit()
        else:
            sp = math.hypot(x[2], x[3])
            if sp > 0.30:
                self.heading = (x[2] / sp, x[3] / sp)
        rho, (h0, h1) = self.predicted_range(ax, ay, dz)
        H = [0.0] * n
        H[0], H[1] = h0, h1
        H[4] = 1.0 if self.estimate_bias else 0.0
        if k is not None and k < (self.ib if self.ib is not None else self.n_off):
            H[tracker.N_STATE + k] = 1.0
        if self.ib is not None:
            H[self.ib] = self.shade(ax, ay)
        nz = [j for j in range(n) if H[j]]
        y = z - (rho + sum(H[j] * x[j] for j in nz if j >= 4))
        PHt = [sum(P[i][j] * H[j] for j in nz) for i in range(n)]
        HPHt = sum(H[j] * PHt[j] for j in nz)
        R = sigma * sigma
        S = HPHt + R
        nu = y / math.sqrt(S)
        self._zupt_mode(nu)
        if abs(nu) > gate:
            return False, y, nu, S
        if self.STILL_LONG and self.ZUPT and self.still and nu > self.STILL_LONG:
            return False, y, nu, S           # standing: a range too long is a body, drop it
        kh = huber_k_long if (nu > 0 and huber_k_long > 0) else huber_k
        if abs(nu) > kh:
            R *= abs(nu) / kh
            S = HPHt + R
        K = [PHt[i] / S for i in range(n)]
        for i in range(n):
            x[i] += K[i] * y
        IKH = [[(1.0 if i == j else 0.0) - K[i] * H[j] for j in range(n)] for i in range(n)]
        A = [[sum(IKH[i][q] * P[q][j] for q in range(n)) for j in range(n)] for i in range(n)]
        self.P = [[sum(A[i][q] * IKH[j][q] for q in range(n)) + K[i] * R * K[j]
                   for j in range(n)] for i in range(n)]
        lo, hi = tracker.BIAS_LIMIT
        x[4] = min(max(x[4], lo), hi)
        for i in range(tracker.N_STATE, n):
            if i == self.ib:
                x[i] = min(max(x[i], BETA_LIMIT[0]), BETA_LIMIT[1])
            else:
                x[i] = min(max(x[i], -tracker.ANCHOR_OFF_LIMIT), tracker.ANCHOR_OFF_LIMIT)
        return True, y, nu, S


# Body shadow from ANY direction ("R"): one more state per anchor, rho_i >= 0,
# the range a body standing between the tag and anchor i adds right now. Unlike
# B it assumes nothing about where the body is; each rho_i fades back to 0
# (the shopper walks on) unless the ranges keep asking for it.
RHO_SIGMA0, RHO_WALK, RHO_TAU = 0.10, 0.08, 3.0     # prior (m), walk (m/sqrt(s)), fade (s)
RHO_LIMIT = 0.6


class RandBodyFilter(BodyFilter):
    """BodyFilter without beta but with rho_1..rho_n after the d_i: state
    [px py vx vy b d_1..d_n rho_1..rho_n]; the engine sets n_off = 2n."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.n_anchor = self.n_off // 2
        self.ir = tracker.N_STATE + self.n_anchor        # index of rho_1
        for i in range(self.ir, len(self.x)):
            self.x[i] = 0.0
            for q in range(len(self.x)):
                self.P[i][q] = self.P[q][i] = 0.0
            self.P[i][i] = RHO_SIGMA0 ** 2

    def predict(self, t_ms):
        dt = min(max((t_ms - self.t) / 1000.0, 0.0), 2.0)
        if self.ZUPT:
            self.sigma_a = ZUPT_SIGMA_A if self.still else self.sigma_a_move
        ORIG_FILTER.predict(self, t_ms)     # the server's own (tracker.TagFilter is patched in runs)
        n = len(self.x)
        if dt > 0:
            a = math.exp(-dt / RHO_TAU)
            for i in range(self.ir, n):
                self.x[i] *= a
                for q in range(n):
                    self.P[i][q] *= a
                    self.P[q][i] *= a
                self.P[i][i] += (RHO_WALK ** 2 - tracker.ANCHOR_OFF_WALK ** 2) * dt
        if self.FREEZE:
            for i in range(tracker.N_STATE, self.ir):
                for q in range(n):
                    self.P[i][q] = self.P[q][i] = 0.0
        if self.ZUPT and self.still and dt > 0:
            self._linear(2, 0.0, ZUPT_SIGMA_V ** 2)
            self._linear(3, 0.0, ZUPT_SIGMA_V ** 2)

    def update(self, ax, ay, dz, z, sigma, huber_k, gate, k=None, huber_k_long=0.0):
        x, P = self.x, self.P
        n = len(x)
        rho, (h0, h1) = self.predicted_range(ax, ay, dz)
        H = [0.0] * n
        H[0], H[1] = h0, h1
        H[4] = 1.0 if self.estimate_bias else 0.0
        if k is not None and k < self.n_anchor:
            H[tracker.N_STATE + k] = 1.0
            H[self.ir + k] = 1.0
        nz = [j for j in range(n) if H[j]]
        y = z - (rho + sum(H[j] * x[j] for j in nz if j >= 4))
        PHt = [sum(P[i][j] * H[j] for j in nz) for i in range(n)]
        HPHt = sum(H[j] * PHt[j] for j in nz)
        R = sigma * sigma
        S = HPHt + R
        nu = y / math.sqrt(S)
        self._zupt_mode(nu)
        if abs(nu) > gate:
            return False, y, nu, S
        if self.STILL_LONG and self.ZUPT and self.still and nu > self.STILL_LONG:
            return False, y, nu, S
        kh = huber_k_long if (nu > 0 and huber_k_long > 0) else huber_k
        if abs(nu) > kh:
            R *= abs(nu) / kh
            S = HPHt + R
        K = [PHt[i] / S for i in range(n)]
        for i in range(n):
            x[i] += K[i] * y
        IKH = [[(1.0 if i == j else 0.0) - K[i] * H[j] for j in range(n)] for i in range(n)]
        A = [[sum(IKH[i][q] * P[q][j] for q in range(n)) for j in range(n)] for i in range(n)]
        self.P = [[sum(A[i][q] * IKH[j][q] for q in range(n)) + K[i] * R * K[j]
                   for j in range(n)] for i in range(n)]
        lo, hi = tracker.BIAS_LIMIT
        x[4] = min(max(x[4], lo), hi)
        for i in range(tracker.N_STATE, self.ir):
            x[i] = min(max(x[i], -tracker.ANCHOR_OFF_LIMIT), tracker.ANCHOR_OFF_LIMIT)
        for i in range(self.ir, n):
            x[i] = min(max(x[i], 0.0), RHO_LIMIT)       # a body only lengthens a range
        return True, y, nu, S


# ---------------------------------------------------------------------------
# stop refinement
# ---------------------------------------------------------------------------

def solve(fixes, x0, y0, tz, w=None, iters=15):
    """Weighted Gauss-Newton for x, y. fixes: [(ax, ay, az, range)]."""
    x, y = x0, y0
    for _ in range(iters):
        a00 = a01 = a11 = g0 = g1 = 0.0
        for k, (ax, ay, az, r) in enumerate(fixes):
            wk = 1.0 if w is None else w[k]
            dx, dy, dz = x - ax, y - ay, tz - az
            rho = math.sqrt(dx * dx + dy * dy + dz * dz) or 1e-6
            hx, hy = dx / rho, dy / rho
            res = r - rho
            a00 += wk * hx * hx; a01 += wk * hx * hy; a11 += wk * hy * hy
            g0 += wk * hx * res; g1 += wk * hy * res
        det = a00 * a11 - a01 * a01
        if abs(det) < 1e-9:
            break
        sx = (a11 * g0 - a01 * g1) / det
        sy = (a00 * g1 - a01 * g0) / det
        x += sx
        y += sy
        if abs(sx) + abs(sy) < 1e-5:
            break
    res = [r - math.sqrt((x - ax) ** 2 + (y - ay) ** 2 + (tz - az) ** 2)
           for ax, ay, az, r in fixes]
    return x, y, res


def quantile(sorted_vals, q):
    """Linear-interpolated q-quantile of an already sorted list."""
    pos = q * (len(sorted_vals) - 1)
    i = int(pos)
    j = min(i + 1, len(sorted_vals) - 1)
    return sorted_vals[i] + (pos - i) * (sorted_vals[j] - sorted_vals[i])


def rms(v):
    return math.sqrt(sum(e * e for e in v) / len(v)) if v else 0.0


class Refiner:
    """Re-solves the position from the stop's median ranges (see module doc)."""

    def __init__(self, eng, scene, mode, tau=0.035, min_n=3):
        self.eng, self.mode, self.tau, self.min_n = eng, mode, tau, min_n
        self.q, self.q_corr = 0.5, 0.0       # statistic of each anchor's ranges
        self.tz = scene["tag_z"]
        self.buf = {}                        # anchor -> [(t, corrected range)]
        self.heading = None                  # last direction of travel
        eng.on_sample = self.on_sample

    def on_sample(self, tag, s):
        t, anchor, raw = s[0], s[1], s[2]
        b = self.buf.setdefault(anchor, [])
        b.append((t, self.eng.correct(anchor, raw)))
        while b and t - b[0][0] > 20000:
            del b[0]

    def refine(self, tag, est):
        x, y = est["x"], est["y"]
        sp = math.hypot(est["vx"], est["vy"])
        if sp > 0.30:
            self.heading = (est["vx"] / sp, est["vy"] / sp)
        tr = self.eng.tracks.get(tag)
        if (self.mode == "G" or not est["valid"] or not est["still"] or tr is None
                or tr.f is None or tr.still_from is None):
            return x, y
        fx = tr.f.x
        n_off = tr.f.n_off
        beta = 0.0
        if getattr(tr.f, "ib", None) is not None:      # body-shadow EKF
            beta = fx[tr.f.ib]
        fixes, raw, n_s = [], [], []
        for anchor, buf in self.buf.items():
            vals = sorted(z for t, z in buf if t >= tr.still_from)
            pos = self.eng.anchors.get(anchor)
            if pos is None or len(vals) < self.min_n:
                continue
            k = len(vals)
            if self.q == 0.5:
                med = vals[k // 2] if k % 2 else 0.5 * (vals[k // 2 - 1] + vals[k // 2])
            else:                            # lower quantile: NLOS only lengthens
                med = quantile(vals, self.q) + self.q_corr
            d = 0.0
            if n_off:
                i = self.eng.anchor_index.get(anchor)
                d = fx[tracker.N_STATE + i] if i is not None else 0.0
            shade = tr.f.shade(pos[0], pos[1]) if beta else 0.0
            fixes.append((pos[0], pos[1], pos[2], med - fx[4] - d - shade * beta))
            raw.append((pos[0], pos[1], pos[2], med - fx[4] - d))
            n_s.append(k)
        if len(fixes) < 3:
            return x, y
        if "HT" in self.mode and beta > 0.03 and len(raw) >= 4 and min(n_s) >= self.min_n + 1:
            fixes = self._heading_test(tr.f, raw, x, y, beta) or fixes
            if "S" not in self.mode:
                return x, y
        w = None
        if "HP" in self.mode and self.heading:
            w = []
            for ax, ay, _az, _r in fixes:
                ux, uy = ax - x, ay - y
                un = math.hypot(ux, uy) or 1.0
                behind = (ux * self.heading[0] + uy * self.heading[1]) / un < -0.35
                w.append(0.25 if behind else 1.0)
        if "RW" in self.mode:
            return self._rw(fixes, x, y)
        sx, sy, res = solve(fixes, x, y, self.tz, w)
        if "FDE" in self.mode:
            sx, sy = self._fde(fixes, sx, sy, res, w)
        if math.hypot(sx - x, sy - y) > 0.5:          # implausible: keep the EKF
            return x, y
        return sx, sy

    HT_STEPS = 24                    # candidate headings, every 15 degrees

    def _heading_test(self, f, raw, x, y, beta):
        """Which way is the trolley facing? Each candidate heading predicts
        which anchors the body shadows; the one whose corrected median ranges
        fit one position best wins (a small prior keeps the track's heading
        when they fit alike). A clearly better one replaces the filter's
        heading, e.g. after a short leg round a corner, too short for the
        track to show. Returns the fixes corrected with the chosen heading."""
        def fit(h):
            fx = [(ax, ay, az, r - beta * f.shade(ax, ay, h, (x, y))) for ax, ay, az, r in raw]
            _sx, _sy, res = solve(fx, x, y, self.tz)
            return rms(res), fx

        cur = f.heading
        best_rms, best_fx = fit(cur) if cur else (float("inf"), None)
        cur_rms, best_h = best_rms, cur
        for k in range(self.HT_STEPS):
            a = 2 * math.pi * k / self.HT_STEPS
            h = (math.cos(a), math.sin(a))
            r, fx = fit(h)
            if cur:
                r += 0.005 * (1 - (h[0] * cur[0] + h[1] * cur[1])) / 2
            if r < best_rms:
                best_rms, best_fx, best_h = r, fx, h
        if best_h is not cur and (cur is None or best_rms < 0.7 * cur_rms):
            f.heading = best_h
            f.P[0][0] += 0.05 ** 2           # let the EKF move to the new fit
            f.P[1][1] += 0.05 ** 2
            return best_fx
        return None

    def _fde(self, fixes, x, y, res, w):
        used = list(range(len(fixes)))
        cur = rms(res)
        while len(used) >= 4 and cur > self.tau:
            best = None
            for j in used:
                sub = [i for i in used if i != j]
                ws = [w[i] for i in sub] if w else None
                xs, ys, rs = solve([fixes[i] for i in sub], x, y, self.tz, ws)
                ax, ay, az, r = fixes[j]
                rj = r - math.sqrt((xs - ax) ** 2 + (ys - ay) ** 2 + (self.tz - az) ** 2)
                if rj > 0.03 and (best is None or rms(rs) < best[0]):
                    best = (rms(rs), j, xs, ys)
            if best is None or best[0] > 0.6 * cur:
                break
            cur, j, x, y = best
            used.remove(j)
        return x, y

    def _rw(self, fixes, x0, y0):
        num_x = num_y = den = 0.0
        n = len(fixes)
        for size in range(3, n + 1):
            for sub in itertools.combinations(range(n), size):
                xs, ys, rs = solve([fixes[i] for i in sub], x0, y0, self.tz)
                rn = sum(e * e for e in rs) / max(size - 2, 1)
                wt = 1.0 / max(rn, 1e-6)
                num_x += wt * xs; num_y += wt * ys; den += wt
        return (num_x / den, num_y / den) if den else (x0, y0)


# ---------------------------------------------------------------------------
# runs
# ---------------------------------------------------------------------------

# The components added one at a time (each row = the row above + one more),
# and the final pipeline without the static calibration.
LADDER = [
    ("G", "EKF sekarang (server, algoritma G)"),
    ("K+F", "+ kalibrasi statis offset tiap anchor (dipercaya)"),
    ("K+F+B+hd", "+ state bayangan badan β dan arah hadap"),
    ("K+F+B+hd+Z", "+ mode diam (ZUPT) = BZ-EKF (final)"),
    ("K+F+B+hd+Z+S+FDE", "BZ-EKF + hitung ulang saat berhenti + buang anchor janggal (diuji, tidak dipakai)"),
    ("B+hd+Z", "BZ-EKF tanpa kalibrasi statis (offset dipelajari sendiri)"),
    ("K+F+B+imu+Z", "BZ-EKF dengan arah hadap dari gyroscope (IMU) di tag"),
]
FINAL = "K+F+B+hd+Z"
FINAL_IMU = "K+F+B+imu+Z"
MODES = [m for m, _ in LADDER]
ORIG_FILTER, ORIG_TRACK = tracker.TagFilter, tracker.TagTrack
REFINE = ("S", "FDE", "RW", "HP", "HT")


class CalTrack(ORIG_TRACK):
    """A track whose offsets are known from the static calibration."""

    def __init__(self, tag_id):
        super().__init__(tag_id)
        self.bias_known = True


def run(scene, stream, mode, tau=0.035, cal=None, probe=None, imu=None):
    """mode: tokens joined by "+":
      B   body-shadow state beta        K    static calibration of the offsets
      hd  heading from displacement     sym  symmetric Huber (beta explains NLOS)
      Z   zero-velocity mode at stops   F    with K: offsets frozen at calibration
      W   longer window at a stop       S/FDE/RW/HP/HT  stop refinement (Refiner)
      imu heading from a gyro on the tag (imu: t_ms -> heading) instead of hd"""
    tok = set(mode.split("+"))
    custom = bool(tok & {"B", "K", "Z"})
    tracker.TagFilter = BodyFilter if custom else ORIG_FILTER
    tracker.TagTrack = CalTrack if "K" in tok else ORIG_TRACK
    BodyFilter.USE_BETA = "B" in tok
    BodyFilter.HEAD_DISP = "hd" in tok
    BodyFilter.CALIBRATED = "K" in tok
    BodyFilter.ZUPT = "Z" in tok
    BodyFilter.FREEZE = "F" in tok
    BodyFilter.IMU = staticmethod(imu.heading) if imu and "imu" in tok else None
    BodyFilter.IMU_STILL = staticmethod(imu.still) if imu and "imu" in tok else None
    sc = json.loads(json.dumps(scene))
    trk = sc.setdefault("tracking", {})
    if "W" in tok:
        trk.update({"window_ms": 10000, "post_window": 50})
    if "sym" in tok:
        trk["huber_k_long"] = 0.0
    if "K" in tok and cal:
        sc["calibration"] = {"anchors": {aid: {"bias": v, "scale": 1.0} for aid, v in cal.items()}}
    eng = tracker_lab.LabEngine()
    eng.configure(sc, True, nlos_mod=sim.scene_mod)
    if "B" in tok:
        eng.n_off = len(eng.anchor_index) + 1          # one more state: beta
    base = "+".join(t for t in mode.split("+") if t in REFINE) or "G"
    ref = Refiner(eng, scene, base, tau)
    track = []
    for recv, dev, payload in stream:
        for tag in eng.ingest(dev, payload, recv):
            est = eng.estimate(tag, recv)
            if est:
                x, y = ref.refine(tag, est)
                if probe:
                    probe(eng, tag, est, recv, x, y)
                track.append((recv, x, y, est["valid"], est["t"], est["still"],
                              est["vx"], est["vy"]))
    tracker.TagFilter, tracker.TagTrack = ORIG_FILTER, ORIG_TRACK
    return track


BINS = [(1, 2), (2, 3), (3, 4), (4, 6.5)]      # seconds since the trolley stopped


def metrics(segs, track, dur, warmup=3.0):
    """Errors sampled every 100 ms. At stops (from 1 s after stopping) and per
    time since the stop; while moving, against the true position now."""
    stop, move, bins = [], [], {b: [] for b in BINS}
    j = 0
    end = segs[-1][1]
    t = track[0][0] + warmup * 1000
    while t < end * 1000:
        while j + 1 < len(track) and track[j + 1][0] <= t:
            j += 1
        tx, ty = sim.truth_at(segs, t / 1000)
        e = math.hypot(track[j][1] - tx, track[j][2] - ty)
        mv, since = st.moving_at(segs, t / 1000)
        if mv:
            move.append(e)
        elif since > 1.0:
            stop.append(e)
            for b in BINS:
                if b[0] <= since < b[1]:
                    bins[b].append(e)
        t += 100
    return stop, move, bins


def make_case(name, seed, n_anchors=4):
    """One simulated walk: the telemetry stream every variant replays, and the
    static calibration (true offsets + CAL_SIGMA error) the K variants use."""
    rng = random.Random(seed)
    w = world(name, rng)
    scene = scene_for(n_anchors)
    segs = st.build_segs(WALK, 0.6, 6.0, 6.0)
    dur = segs[-1][1] + 1.0
    ex, bias = radio(scene, segs, rng, dur, w)
    stream = sim.telemetry_stream(scene, ex, random.Random(rng.random()), dur, "1.0.32")
    crng = random.Random(seed * 7 + 1)
    cal = {aid: v + crng.gauss(0, CAL_SIGMA) for aid, v in sorted(bias.items())}
    return scene, segs, dur, stream, cal, w


def one(job):
    name, seed, modes, n_anchors = job
    scene, segs, dur, stream, cal, _w = make_case(name, seed, n_anchors)
    out = {}
    for mode in modes:
        tr = run(scene, stream, mode, cal=cal, imu=imu_model(segs, seed))
        stop, move, bins = metrics(segs, tr, dur)
        sc = st.score(segs, tr, dur)
        out[mode] = {"stop": stop, "move": move, "bins": bins, "sync": sc["sync_move"],
                     "avail": sc["avail"]}
    return n_anchors, name, out


def pct(v, q):
    v = sorted(v)
    return v[min(int(q * (len(v) - 1) + 0.5), len(v) - 1)] if v else float("nan")


def mean(v):
    return sum(v) / len(v) if v else float("nan")


def summarize(res, n_anchors, name, mode):
    runs = [o[mode] for na, n, o in res if na == n_anchors and n == name and mode in o]
    stop = [e for r in runs for e in r["stop"]]
    settled = [e for r in runs for b in BINS[1:] for e in r["bins"][b]]
    per_run = [mean(r["stop"]) for r in runs]
    m = mean(per_run)
    return {
        "stop_mean": mean(stop), "stop_med": pct(stop, 0.5), "stop_p95": pct(stop, 0.95),
        "settled_mean": mean(settled), "settled_med": pct(settled, 0.5),
        "bins": {f"{b[0]}-{b[1]:g}": mean([e for r in runs for e in r["bins"][b]]) for b in BINS},
        "runs_sd": math.sqrt(sum((x - m) ** 2 for x in per_run) / max(len(per_run) - 1, 1)),
        "move_mean": mean([e for r in runs for e in r["move"]]),
        "sync_move": mean([r["sync"] for r in runs]),
        "avail": mean([r["avail"] for r in runs]),
        "within4": sum(e <= 0.04 for e in stop) / len(stop) if stop else float("nan"),
        "cdf": [pct(stop, q / 100) for q in range(1, 100)],
        "runs": len(runs),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--seed", type=int, default=700)
    ap.add_argument("--scenarios", default="market,market-los,robust")
    ap.add_argument("--modes", default=",".join(MODES))
    ap.add_argument("--anchors", default="4,6", help="4, 6 or 4,6")
    ap.add_argument("--modes6", default=",".join(("G", FINAL, FINAL_IMU)),
                    help="modes run with 6 anchors")
    ap.add_argument("--out", default=str(ROOT / "docs" / "hasil-simulasi" / "riset_akurasi.json"))
    a = ap.parse_args()
    names = a.scenarios.split(",")
    sets = {4: a.modes.split(","), 6: a.modes6.split(",")}
    anchors = [int(x) for x in a.anchors.split(",")]
    jobs = [(n, a.seed + k, sets[na], na) for na in anchors for n in names for k in range(a.runs)]
    with Pool(8) as pool:
        res = pool.map(one, jobs)
    summary = {str(na): {n: {m: summarize(res, na, n, m) for m in sets[na]} for n in names}
               for na in anchors}
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps({
        "runs": a.runs, "seed": a.seed, "laps": LAPS, "ladder": LADDER, "bins": [list(b) for b in BINS],
        "summary": summary}, indent=1), encoding="utf-8")
    for na in anchors:
        for name in names:
            print(f"\n== {name}, {na} anchor  (cm; diam = sejak 1 s setelah berhenti)")
            print(f"{'algoritma':20s} {'diam':>6s} {'median':>6s} {'P95':>6s} {'>=2s':>6s} "
                  f"{'<=4cm':>6s} {'sd run':>6s} {'gerak':>6s}")
            for mode in sets[na]:
                r = summary[str(na)][name][mode]
                print(f"{mode:20s} {r['stop_mean'] * 100:6.1f} {r['stop_med'] * 100:6.1f} "
                      f"{r['stop_p95'] * 100:6.1f} {r['settled_mean'] * 100:6.1f} "
                      f"{r['within4'] * 100:5.0f}% {r['runs_sd'] * 100:6.1f} {r['move_mean'] * 100:6.1f}")
    print("\nwrote", a.out)


if __name__ == "__main__":
    main()
