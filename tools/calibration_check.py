"""Check the stored calibration points before pressing Apply (the same rules
the server applies, server/calibration.py; Setup -> Calibrate shows them too).

    python tools/calibration_check.py                      # live server, 127.0.0.1:8080
    python tools/calibration_check.py --server http://192.168.0.5:8080

Per point, in the order they were measured: every anchor's offset (measured -
true range) and what is wrong with it, then what Apply would do: which ranges
it leaves out, how far each point would be off with the correction from the
other points, and whether it accepts or refuses.
"""
import argparse
import json
import sys
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "server"))
import calibration as calib  # noqa: E402


def get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.loads(r.read())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--server", default="http://127.0.0.1:8080")
    a = ap.parse_args()
    pts = (get(a.server + "/api/v1/calibration")["calibration"].get("points") or [])
    scene = get(a.server + "/api/v1/scene")
    scene = scene.get("scene", scene)
    anchors = {x["id"]: (x["x"], x["y"], x.get("z", 2.2)) for x in scene["anchors"]}
    if not pts:
        print("no calibration points stored")
        return
    print(f"{len(pts)} point(s); anchors: " + ", ".join(f"{k} ({v[0]}, {v[1]}, z {v[2]})"
                                                        for k, v in sorted(anchors.items())))
    for k, p in enumerate(pts):
        print(f"\npoint {k + 1}: X {p['x']:.3f}  Y {p['y']:.3f}  (tag height {p.get('tag_z')} m)")
        chk = calib.check_point(pts, k, anchors)
        for aid, v in sorted(p["anchors"].items()):
            notes = chk["anchors"].get(aid) or []
            print(f"  {aid}: true {v['true']:.3f} m, measured {v['median']:.3f} m, offset "
                  f"{(v['median'] - v['true']) * 100:+.1f} cm, spread {v.get('std', 0) * 100:.0f} cm"
                  + ("   <-- " + "; ".join(notes) if notes else "   ok"))
        for w in chk["warnings"]:
            if not any(w.endswith(n) for n in sum(chk["anchors"].values(), [])):
                print("  " + w)
    res = calib.fit(pts, anchors)
    print("\nwhat Apply would do:")
    for d in res["dropped"]:
        print(f"  leaves out point {d['point']} {d['anchor']}: {d['reason']}")
    for c in res["check"]:
        e = "-" if c["error"] is None else f"{c['error'] * 100:.1f} cm"
        b = "-" if c["without"] is None else f"{c['without'] * 100:.1f} cm"
        print(f"  point {c['point']}: {e} off with the other points' correction ({b} without calibration)")
    for w in res["warnings"]:
        print("  note: " + w)
    if res["refused"]:
        print("\nREFUSED, nothing would change:")
        for r in res["refused"]:
            print("  - " + r)
    else:
        print("\nOK: Apply would be accepted")


if __name__ == "__main__":
    main()
