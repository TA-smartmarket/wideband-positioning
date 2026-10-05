"""Room geometry, obstacles and LOS/NLOS classification.

Coordinate system (right-handed, Z up) — the same everywhere in this project:

    X : room width   (metres, 0 .. room.width)
    Y : room depth   (metres, 0 .. room.depth)
    Z : height       (metres, 0 = floor)

The web UI maps this to Three.js as (x, z, y) so Y is up on screen.

Obstacles are **oriented boxes** (yaw rotation around Z). They are not just
decoration: every anchor→tag path is tested against them, and a blocked path is
treated as NLOS — the measurement noise for that anchor is inflated (and an
optional positive bias is added), which makes the EKF down-weight or reject it.
This is standard NLOS identification/mitigation for UWB.
"""
import math

DEFAULT_ROOM = {"width": 5.0, "depth": 4.0, "height": 2.7}
# Height of the tag above the floor, in metres. This is NOT a guess any more:
# it is a scene field the operator sets, because the 3D→horizontal projection
# below divides by it and a wrong value shifts every position (30 cm of error
# moves a range by ~13 cm). The default matches how the tag is normally held.
DEFAULT_TAG_Z = 1.3


def default_scene():
    return {"room": dict(DEFAULT_ROOM), "anchors": [], "obstacles": [],
            "tag_z": DEFAULT_TAG_Z}


def _norm_room(room):
    out = dict(DEFAULT_ROOM)
    if isinstance(room, dict):
        for k in ("width", "depth", "height"):
            try:
                v = float(room.get(k, out[k]))
                if v > 0:
                    out[k] = v
            except (TypeError, ValueError):
                pass
    return out


def _norm_obstacle(ob, idx=0):
    def f(k, d=0.0):
        try:
            return float(ob.get(k, d))
        except (TypeError, ValueError):
            return d
    return {
        "id": str(ob.get("id") or f"obstacle-{idx + 1}"),
        "label": str(ob.get("label") or "Obstacle"),
        "x": f("x"), "y": f("y"), "z": f("z"),
        "sx": max(f("sx", 0.2), 0.01),
        "sy": max(f("sy", 0.2), 0.01),
        "sz": max(f("sz", 0.2), 0.01),
        "rot": f("rot"),                       # degrees, yaw around Z
        "atten": max(f("atten", 1.0), 0.0),    # 0 = transparent, 1 = default wall
    }


def _norm_tag_z(raw):
    try:
        v = float(raw.get("tag_z", DEFAULT_TAG_Z))
    except (TypeError, ValueError):
        return DEFAULT_TAG_Z
    # Must stay below the anchors and above the floor: a tag at or under 0
    # would make the projection divide by zero, and one above the ceiling is
    # physically impossible.
    return min(max(v, 0.05), 3.0)


def normalise_scene(raw):
    """Coerce an arbitrary payload into a valid scene dict."""
    raw = raw if isinstance(raw, dict) else {}
    room = _norm_room(raw.get("room"))
    anchors = []
    for i, a in enumerate(raw.get("anchors") or []):
        if not isinstance(a, dict):
            continue
        try:
            anchors.append({
                "id": str(a.get("id") or f"anchor-{i + 1}"),
                "label": str(a.get("label") or a.get("id") or f"anchor-{i + 1}"),
                "x": float(a.get("x", 0.0)),
                "y": float(a.get("y", 0.0)),
                "z": float(a.get("z", 2.2)),
            })
        except (TypeError, ValueError):
            continue
    obstacles = [_norm_obstacle(o, i) for i, o in enumerate(raw.get("obstacles") or [])
                 if isinstance(o, dict)]
    return {"room": room, "anchors": anchors, "obstacles": obstacles,
            "tag_z": _norm_tag_z(raw)}


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------

def segment_hits_box(p0, p1, ob):
    """3D segment vs oriented box, slab method in the box's local frame."""
    hx, hy, hz = ob["sx"] / 2.0, ob["sy"] / 2.0, ob["sz"] / 2.0
    if hx <= 0 or hy <= 0 or hz <= 0:
        return False

    rot = math.radians(ob.get("rot", 0.0))
    c, s = math.cos(-rot), math.sin(-rot)

    def to_local(p):
        dx, dy, dz = p[0] - ob["x"], p[1] - ob["y"], p[2] - ob["z"]
        return (dx * c - dy * s, dx * s + dy * c, dz)

    a = to_local(p0)
    b = to_local(p1)
    d = (b[0] - a[0], b[1] - a[1], b[2] - a[2])

    tmin, tmax = 0.0, 1.0
    for i, half in enumerate((hx, hy, hz)):
        if abs(d[i]) < 1e-9:
            if a[i] < -half or a[i] > half:
                return False
        else:
            t1 = (-half - a[i]) / d[i]
            t2 = (half - a[i]) / d[i]
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return False
    return True


def los_blockers(p0, p1, obstacles):
    """Obstacles intersected by the segment p0→p1 (skips transparent ones)."""
    hits = []
    for ob in obstacles:
        if ob.get("atten", 1.0) <= 0.0:
            continue                      # explicitly transparent
        if segment_hits_box(p0, p1, ob):
            hits.append(ob)
    return hits


def horizontal_range(range3d, dz):
    """Project a 3D anchor→tag range onto the horizontal plane.

    The EKF state is 2D (x, y); anchors sit at a known height, so the vertical
    offset is removed exactly instead of being absorbed as error.
    """
    if range3d <= 0:
        return 0.0
    r2 = range3d * range3d - dz * dz
    if r2 <= 0:
        return max(range3d * 0.1, 0.01)   # degenerate geometry
    return math.sqrt(r2)


def measurement_sigma(blockers, base_sigma, nlos_factor=8.0):
    """Range noise for a path: base when LOS, inflated when blocked."""
    if not blockers:
        return base_sigma
    # the strongest attenuator on the path decides
    atten = max(ob.get("atten", 1.0) for ob in blockers)
    return base_sigma * (1.0 + (nlos_factor - 1.0) * min(atten, 1.0))


def nlos_bias(blockers, bias_m=0.35):
    """UWB NLOS paths read long; add a positive bias (metres)."""
    if not blockers:
        return 0.0
    atten = max(ob.get("atten", 1.0) for ob in blockers)
    return bias_m * min(atten, 1.0)
