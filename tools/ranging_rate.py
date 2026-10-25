"""How fast the hardware really ranges, from recordings (server/logs/*.jsonl).

    python tools/ranging_rate.py server/logs/20261023-*.jsonl
    python tools/ranging_rate.py before.jsonl after.jsonl      # compare two sessions

For every recording, from the ranges the TAG reported (what CALF-UWB uses):

  ranges/s per anchor   measured ranges per second for each anchor
  anchor's own copy     the same anchor's count of ranges it computed itself;
                        more than the tag's = RANGE_REPORTs lost on the way
  rounds/s              ranging rounds that produced at least one range
                        (ranges within 120 ms of the round's first); the
                        library polls every 80 + 3 x N x 7 ms (164 ms with 4
                        anchors), i.e. at most ~5.8 rounds/s
  anchors per round     share of rounds with 1, 2, ... anchors
  gap in polls          time between rounds in poll periods: 1x = no round
                        lost, 2x = one lost, ...
  impossible            ranges longer than the room diagonal + 2.5 m (garbage)
  robust                the tag's ranging-round mode, from its status (fw >= 1.0.36)

No ground truth is needed: this is about the radio, not the position.
"""
import argparse
import json
import math
from collections import Counter
from pathlib import Path

REPLY_US, TIMER_MS = 7000, 80          # DW1000Ranging defaults (lib/DW1000)
ROUND_SPAN_MS = 120


def load(path):
    """Tag and anchor samples as (boot, ts, anchor, raw). A node that restarts
    starts its millis() again: its packets are numbered by boot so every time
    span is taken within one boot."""
    meta, tag, anchor, robust = None, {}, {}, set()
    boot, last_ts = {}, {}
    diag = {"ack": {}, "late": 0, "dropped": {}}     # tag status, fw >= 1.0.37
    for line in open(path, encoding="utf-8"):
        o = json.loads(line)
        if o.get("k") == "meta":
            meta = o
            continue
        if o.get("k") != "rx":
            continue
        dev, p = o.get("dev", ""), o["p"]
        if p.get("ts") is not None:
            if dev in last_ts and p["ts"] < last_ts[dev] - 2000:
                boot[dev] = boot.get(dev, 0) + 1          # restarted
            last_ts[dev] = p["ts"]
        b = boot.get(dev, 0)
        st = p.get("status") or {}
        if dev.startswith("tag"):
            if "robust" in st:
                robust.add(bool(st["robust"]))
            for a, ms in (st.get("ack_ms") or {}).items():
                diag["ack"].setdefault(a, []).append(ms)
            diag["late"] += st.get("ack_late", 0)
            if "anchors_dropped" in st:
                diag["dropped"][b] = max(diag["dropped"].get(b, 0), st["anchors_dropped"])
        for g in p.get("ranges", []):
            if g.get("raw") is None or g.get("ts") is None:
                continue
            a = g.get("src")
            key = (b, a, g.get("seq"), g["ts"])
            if dev.startswith("tag"):
                tag[key] = (b, g["ts"], a, float(g["raw"]))
            elif dev == a:
                anchor[key] = (b, g["ts"], a, float(g["raw"]))
    restarts = {d: n for d, n in boot.items() if n}
    return meta, sorted(tag.values()), sorted(anchor.values()), robust, restarts, diag


def pct(v, q):
    v = sorted(v)
    return v[min(int(q * (len(v) - 1) + 0.5), len(v) - 1)]


def span_s(samples):
    """Seconds covered by (boot, ts, ...) samples, summed over the boots."""
    first, last = {}, {}
    for b, t, *_ in samples:
        first[b] = min(first.get(b, t), t)
        last[b] = max(last.get(b, t), t)
    return sum(last[b] - first[b] for b in first) / 1000


def stats(path):
    meta, tag, anchor, robust, restarts, diag = load(path)
    if not tag:
        return None
    room = (meta or {}).get("scene", {}).get("room", {})
    limit = math.hypot(room.get("width", 10), room.get("depth", 10)) + 2.5
    anchors = sorted({a for _b, _t, a, _r in tag})
    n = len(anchors)
    poll = TIMER_MS + 3 * n * REPLY_US / 1000
    dur = span_s(tag) or 1.0
    per_tag = Counter(a for _b, _t, a, _r in tag)
    own = {}
    for a in anchors:                 # every node has its own millis(): span per anchor
        mine = [r for r in anchor if r[2] == a]
        if span_s(mine) > 1:
            own[a] = len(mine) / span_s(mine)
    rounds, cur = [], []
    for b, t, a, _r in tag:
        if cur and (b != cur[0][0] or t - cur[0][1] > ROUND_SPAN_MS):
            rounds.append(cur)
            cur = []
        cur.append((b, t, a))
    rounds.append(cur)
    size = Counter(len({a for _b, _t, a in r}) for r in rounds)
    gaps = [rounds[i + 1][0][1] - rounds[i][0][1] for i in range(len(rounds) - 1)
            if rounds[i + 1][0][0] == rounds[i][0][0]]
    gap = Counter(min(round(g / poll), 4) for g in gaps)
    return {
        "name": (meta or {}).get("name") or Path(path).stem,
        "duration_s": dur, "anchors": anchors, "poll_ms": poll,
        "rate": {a: per_tag[a] / dur for a in anchors},
        "own": own,
        "rounds_s": len(rounds) / dur, "max_rounds_s": 1000 / poll * 20 / 21,
        "per_round": {k: v / len(rounds) for k, v in sorted(size.items())},
        "gap": {k: v / len(gaps) for k, v in sorted(gap.items())} if gaps else {},
        "impossible": sum(r > limit for _b, _t, _a, r in tag) / len(tag),
        "restarts": restarts,
        "ack": {a: (pct(v, 0.5), pct(v, 0.9), max(v)) for a, v in sorted(diag["ack"].items())},
        "ack_late": diag["late"],
        "dropped": sum(diag["dropped"].values()) if diag["dropped"] else None,
        "robust": "on" if robust == {True} else "off" if robust == {False} else
                  "mixed" if robust else "unknown (fw < 1.0.36)",
    }


def show(s):
    mean = sum(s["rate"].values()) / len(s["rate"])
    print(f"\n== {s['name']}  ({s['duration_s']:.0f} s, robust round: {s['robust']})")
    if s["restarts"]:
        print("  restarted during the recording (times counted per boot): "
              + ", ".join(f"{d} {n}x" for d, n in sorted(s["restarts"].items())))
    print("  ranges/s per anchor (tag)  : " + ", ".join(f"{a} {v:.2f}" for a, v in s["rate"].items())
          + f"   mean {mean:.2f}")
    if s["own"]:
        print("  anchor's own copy          : " + ", ".join(f"{a} {v:.2f}" for a, v in s["own"].items()))
    print(f"  rounds/s                   : {s['rounds_s']:.2f} of at most {s['max_rounds_s']:.2f}"
          f" (poll every {s['poll_ms']:.0f} ms)")
    print("  anchors per round          : " + ", ".join(f"{k}: {v * 100:.0f}%" for k, v in s["per_round"].items()))
    print("  gap between rounds (polls) : " + ", ".join(f"{k}x: {v * 100:.0f}%" for k, v in s["gap"].items()))
    print(f"  impossible ranges          : {s['impossible'] * 100:.1f}%")
    if s["ack"]:
        print("  POLL_ACK after POLL (ms)   : " + ", ".join(
            f"{a} {m:.0f}/{p90:.0f}/{mx:.0f}" for a, (m, p90, mx) in s["ack"].items())
            + "   (median/90%/max)")
        print(f"  POLL_ACK too late          : {s['ack_late']}   anchors dropped: {s['dropped']}")
    return mean


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("logs", nargs="+")
    a = ap.parse_args()
    rows = []
    for p in a.logs:
        s = stats(p)
        if s is None:
            print(f"\n== {p}: no tag ranges with raw values (fw < 1.0.32?)")
            continue
        rows.append((s, show(s)))
    if len(rows) > 1:
        print(f"\n{'recording':40s} {'robust':>8s} {'ranges/s':>9s} {'rounds/s':>9s} {'4 of 4':>7s} {'impossible':>11s}")
        for s, mean in rows:
            full = s["per_round"].get(len(s["anchors"]), 0.0)
            print(f"{s['name'][:40]:40s} {s['robust'][:8]:>8s} {mean:9.2f} {s['rounds_s']:9.2f} "
                  f"{full * 100:6.0f}% {s['impossible'] * 100:10.1f}%")


if __name__ == "__main__":
    main()
