"""Static range calibration (layer K of CALF-UWB), made safe.

A calibration point is the tag standing still at an X/Y measured with a tape:
per anchor, the distance it should have from there ("true") and the median raw
range measured during the window ("median", with its spread "std"). Every
DW1000 adds its own offset (antenna delay), so for a good set of points
measured - true is about the same for one anchor at every point.

check_point()  how a new point agrees with the points before it; Setup shows
               the warnings right after "Measure here".
fit()          the correction Apply stores, or the reasons it refuses:
               - a range that scattered more than NOISY at a point is not used;
               - with 3+ usable points, a point whose offset is more than
                 OUTLIER from the anchor's median is dropped (a blocked path
                 reads long; a wrong X/Y or anchor position reads either way);
               - offset only, unless 3+ consistent points span SCALE_SPREAD of
                 distance: then a scale too, limited to +-SCALE_LIMIT;
               - two points that disagree by more than OUTLIER cannot be told
                 apart: refused, a third point decides;
               - every point is then located with the correction fitted WITHOUT
                 it (leave one out): more than REFUSE off, or worse on average
                 than with no calibration at all -> refused, nothing changes.
Before this, a single blocked or too-close point gave a wrong scale and threw
the position outside the room.
"""
import math

NOISY = 0.10            # m: spread of one anchor's ranges at one point
OUTLIER = 0.10          # m: offset away from the anchor's median offset
REFUSE = 0.15           # m: leave-one-out position error that refuses Apply
SCALE_POINTS, SCALE_SPREAD, SCALE_LIMIT = 3, 1.0, 0.05
MOVED = 0.02            # m: stored true distance vs the current floor plan


def median(v):
    s = sorted(v)
    n = len(s)
    return 0.0 if n == 0 else (s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2]))


def _obs(points, aid):
    """[(point index, true, median, std)] of one anchor."""
    return [(i, p["anchors"][aid]["true"], p["anchors"][aid]["median"], p["anchors"][aid].get("std", 0.0))
            for i, p in enumerate(points) if aid in (p.get("anchors") or {})]


def _true(p, anchor):
    ax, ay, az = anchor
    return math.sqrt((p["x"] - ax) ** 2 + (p["y"] - ay) ** 2 + (p.get("tag_z", 1.0) - az) ** 2)


def fit_anchor(obs):
    """One anchor's correction from its observations.
    -> (correction or None, [(point index, reason)] dropped, problem or None)"""
    dropped = [(i, f"noisy (spread {s * 100:.0f} cm)") for i, _t, _m, s in obs if s > NOISY]
    good = [(i, t, m) for i, t, m, s in obs if s <= NOISY]
    if not good:
        return None, dropped, "no usable point"
    offs = [m - t for _i, t, m in good]
    med = median(offs)
    if len(good) >= 3:
        keep = []
        for (i, t, m), o in zip(good, offs):
            if abs(o - med) > OUTLIER:
                why = "too long: blocked path?" if o > med else "too short: X/Y or anchor position?"
                dropped.append((i, f"offset {(o - med) * 100:+.0f} cm from the other points ({why})"))
            else:
                keep.append((i, t, m))
        good = keep
    elif len(good) == 2 and abs(offs[0] - offs[1]) > OUTLIER:
        return None, dropped, (f"its 2 points disagree by {abs(offs[0] - offs[1]) * 100:.0f} cm: "
                               "measure a third point, the odd one is then dropped")
    trues = [t for _i, t, _m in good]
    if len(good) >= SCALE_POINTS and max(trues) - min(trues) >= SCALE_SPREAD:
        n = len(good)
        mt = sum(trues) / n
        mm = sum(m for _i, _t, m in good) / n
        sxx = sum((t - mt) ** 2 for t in trues)
        sxy = sum((t - mt) * (m - mm) for _i, t, m in good)
        scale = min(max(sxy / sxx, 1 - SCALE_LIMIT), 1 + SCALE_LIMIT)
        bias = mm - scale * mt
        model = "offset + scale"
    else:
        scale, bias = 1.0, median([m - t for _i, t, m in good])
        model = "offset"
    resid = [m - (scale * t + bias) for _i, t, m in good]
    return ({"bias": round(bias, 4), "scale": round(scale, 5), "points": len(good), "model": model,
             "residual_rms": round(math.sqrt(sum(r * r for r in resid) / len(resid)), 4)},
            dropped, None)


def locate(ranges, anchors, tag_z, common_offset=False):
    """(x, y) from {anchor: range} by Gauss-Newton; common_offset also solves
    one offset shared by every range (what the tracker does uncalibrated)."""
    use = [a for a in ranges if a in anchors]
    if len(use) < (4 if common_offset else 3):
        return None
    x = sum(anchors[a][0] for a in use) / len(use)
    y = sum(anchors[a][1] for a in use) / len(use)
    c = median([ranges[a] - math.hypot(x - anchors[a][0], y - anchors[a][1]) for a in use]) if common_offset else 0.0
    for _ in range(60):
        J, r = [], []
        for a in use:
            ax, ay, az = anchors[a]
            d = math.sqrt((x - ax) ** 2 + (y - ay) ** 2 + (tag_z - az) ** 2) or 1e-6
            r.append(ranges[a] - c - d)
            J.append(((x - ax) / d, (y - ay) / d) + ((1.0,) if common_offset else ()))
        n = len(J[0])
        A = [[sum(j[p] * j[q] for j in J) for q in range(n)] for p in range(n)]
        g = [sum(j[p] * e for j, e in zip(J, r)) for p in range(n)]
        try:
            dx = _solve(A, g)
        except ZeroDivisionError:
            return None
        x, y = x + dx[0], y + dx[1]
        if common_offset:
            c += dx[2]
        if abs(dx[0]) + abs(dx[1]) < 1e-6:
            break
    return x, y


def _solve(A, b):
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for col in range(n):
        piv = max(range(col, n), key=lambda i: abs(M[i][col]))
        if abs(M[piv][col]) < 1e-12:
            raise ZeroDivisionError
        M[col], M[piv] = M[piv], M[col]
        for i in range(n):
            if i != col:
                f = M[i][col] / M[col][col]
                M[i] = [a - f * bb for a, bb in zip(M[i], M[col])]
    return [M[i][n] / M[i][i] for i in range(n)]


def _corrections(points, anchor_ids):
    out = {}
    for aid in anchor_ids:
        corr, _dropped, _problem = fit_anchor(_obs(points, aid))
        if corr:
            out[aid] = corr
    return out


def fit(points, anchors):
    """The correction for every anchor, checked. anchors: {id: (x, y, z)}.
    -> {"anchors", "dropped", "check", "warnings", "refused"}; Apply stores
    "anchors" only when "refused" is empty."""
    refused, warnings, dropped, fitted = [], [], [], {}
    if not points:
        return {"anchors": {}, "dropped": [], "check": [], "warnings": [],
                "refused": ["no calibration points stored"]}
    for k, p in enumerate(points):
        moved = [a for a, v in (p.get("anchors") or {}).items()
                 if a in anchors and abs(_true(p, anchors[a]) - v["true"]) > MOVED]
        if moved:
            refused.append(f"point {k + 1}: {', '.join(sorted(moved))} moved in the floor plan since it was "
                           "measured. Clear and measure again")
    for aid in sorted(anchors):
        corr, drop, problem = fit_anchor(_obs(points, aid))
        dropped += [{"point": i + 1, "anchor": aid, "reason": why} for i, why in drop]
        if problem:
            refused.append(f"{aid}: {problem}")
        elif corr:
            fitted[aid] = corr
    bad = {(d["point"] - 1, d["anchor"]) for d in dropped}
    # what Apply keeps: a dropped range must not judge the other points either
    clean = [dict(p, anchors={a: v for a, v in (p.get("anchors") or {}).items() if (i, a) not in bad})
             for i, p in enumerate(points)]

    # every point located with the correction fitted without it
    check = []
    for k, p in enumerate(points):
        others = clean[:k] + clean[k + 1:]
        corr = _corrections(others, sorted(anchors)) if others else {}
        ok = {a: v for a, v in (p.get("anchors") or {}).items() if a in anchors and (k, a) not in bad}
        ranges = {a: (v["median"] - corr[a]["bias"]) / corr[a]["scale"] for a, v in ok.items() if a in corr}
        tz = p.get("tag_z", 1.0)
        pos = locate(ranges, anchors, tz) if others else None
        base = locate({a: v["median"] for a, v in ok.items()}, anchors, tz, common_offset=True)
        row = {"point": k + 1, "x": round(p["x"], 3), "y": round(p["y"], 3),
               "error": round(math.hypot(pos[0] - p["x"], pos[1] - p["y"]), 4) if pos else None,
               "without": round(math.hypot(base[0] - p["x"], base[1] - p["y"]), 4) if base else None}
        check.append(row)
    errs = [r["error"] for r in check if r["error"] is not None]
    bases = [r["without"] for r in check if r["error"] is not None and r["without"] is not None]
    for r in check:
        if r["error"] is not None and r["error"] > REFUSE:
            refused.append(f"point {r['point']} ({r['x']:.2f}, {r['y']:.2f}) would be {r['error'] * 100:.0f} cm off "
                           "with the correction from the other points. Measure it again or add points")
    if errs and bases and sum(errs) / len(errs) > sum(bases) / len(bases) + 0.02:
        refused.append(f"no better than without calibration ({sum(errs) / len(errs) * 100:.0f} vs "
                       f"{sum(bases) / len(bases) * 100:.0f} cm on average)")
    if len(points) == 1:
        warnings.append("only 1 point: the correction cannot be checked. Measure 2-3 more points")
    elif len(points) == 2:
        warnings.append("2 points: offsets only. 3+ points let a bad point be found and dropped")
    if dropped:
        warnings.append(f"{len(dropped)} range(s) left out, see the list")
    return {"anchors": fitted, "dropped": dropped, "check": check, "warnings": warnings, "refused": refused}


def check_point(points, k, anchors):
    """Warnings for point k, just measured, against the points before it
    (Setup shows them after Measure). With 3+ points the anchor's median over
    all of them is the reference, so a bad EARLIER point is named as such
    instead of making a good new one look wrong."""
    p = points[k]
    earlier = points[:k]
    warn, per = [], {}
    for aid, v in sorted((p.get("anchors") or {}).items()):
        notes = []
        if v.get("std", 0.0) > NOISY:
            notes.append(f"noisy (spread {v['std'] * 100:.0f} cm): something moved or blocked it")
        offs = [(i, q["anchors"][aid]["median"] - q["anchors"][aid]["true"]) for i, q in enumerate(points[:k + 1])
                if aid in (q.get("anchors") or {}) and q["anchors"][aid].get("std", 0.0) <= NOISY]
        off = v["median"] - v["true"]
        if len(offs) >= 3:
            med = median([o for _i, o in offs])
            d = off - med
            if v.get("std", 0.0) <= NOISY and abs(d) > OUTLIER:
                notes.append(f"offset {d * 100:+.0f} cm vs the other points ("
                             + ("too long: blocked path?" if d > 0 else "too short: X/Y or anchor position?") + ")")
            for i, o in offs:
                if i != k and abs(o - med) > OUTLIER:
                    warn.append(f"{aid}: earlier point {i + 1} is the odd one ({(o - med) * 100:+.0f} cm), "
                                "Apply leaves it out")
        elif len(offs) == 2 and v.get("std", 0.0) <= NOISY:
            (i, o), = [x for x in offs if x[0] != k]
            if abs(off - o) > OUTLIER:
                notes.append(f"disagrees with point {i + 1} by {abs(off - o) * 100:.0f} cm: measure another "
                             "point to tell which one is wrong")
        per[aid] = notes
        warn += [f"{aid}: {n}" for n in notes]
    if earlier:
        corr = _corrections(earlier, sorted(anchors))
        ranges = {a: (v["median"] - corr[a]["bias"]) / corr[a]["scale"]
                  for a, v in (p.get("anchors") or {}).items() if a in corr and a in anchors and not per.get(a)}
        pos = locate(ranges, anchors, p.get("tag_z", 1.0))
        if pos and math.hypot(pos[0] - p["x"], pos[1] - p["y"]) > OUTLIER:
            warn.append(f"the ranges put the tag at ({pos[0]:.2f}, {pos[1]:.2f}), "
                        f"{math.hypot(pos[0] - p['x'], pos[1] - p['y']) * 100:.0f} cm from the X/Y entered")
    return {"warnings": warn, "anchors": per, "first": not earlier}
