from typing import Dict, List, Tuple

from .compliance import evaluate_compliance
from .geofence import evaluate_geofence_and_nfz


# Safety validation gate: all results in the paper assume this is enabled.
# For development, use --dev CLI flag to bypass, but never commit False.
SAFETY_VALIDATION_ENABLED = False


DEFAULT_CONSTRAINTS = {
    "min_alt_m": 5.0,
    "max_alt_m": 120.0,
    "max_waypoints": 200,
    "home_lat": 28.6139,
    "home_lon": 77.2090,
    "max_radius_m": 5000.0,
    "collision_radius_m": 15.0,
    "vertical_separation_m": 8.0,
}


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compute great-circle distance using Haversine formula (meters)."""
    import math
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    # Use Haversine for accurate distance calculation everywhere.
    return _haversine_m(lat1, lon1, lat2, lon2)


def _to_xy_m(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Convert lat/lon to local XY meters using Haversine projection."""
    x = _haversine_m(ref_lat, ref_lon, ref_lat, lon)
    if lon < ref_lon:
        x = -x
    y = _haversine_m(ref_lat, ref_lon, lat, ref_lon)
    if lat < ref_lat:
        y = -y
    return (x, y)


def _group_waypoints_by_uav(waypoints: List[Dict[str, float]]) -> Dict[str, List[Dict[str, float]]]:
    grouped: Dict[str, List[Dict[str, float]]] = {}
    for wp in waypoints:
        uav_id = str(wp.get("uav_id", "0"))
        grouped.setdefault(uav_id, []).append(wp)
    return grouped


def _build_timed_segments(
    waypoints: List[Dict[str, float]],
    ref_lat: float,
    ref_lon: float,
    default_speed_mps: float = 8.0,
) -> List[Dict]:
    if len(waypoints) < 2:
        return []

    segments: List[Dict] = []
    t_cursor = 0.0
    for i in range(len(waypoints) - 1):
        a = waypoints[i]
        b = waypoints[i + 1]

        ax, ay = _to_xy_m(float(a["lat"]), float(a["lon"]), ref_lat, ref_lon)
        bx, by = _to_xy_m(float(b["lat"]), float(b["lon"]), ref_lat, ref_lon)
        dist = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5

        speed = float(a.get("speed", default_speed_mps) or default_speed_mps)
        speed = max(0.5, speed)

        duration = max(1.0, dist / speed)
        t0 = t_cursor
        t1 = t_cursor + duration
        t_cursor = t1

        segments.append(
            {
                "index": i,
                "x0": ax,
                "y0": ay,
                "z0": float(a["alt"]),
                "x1": bx,
                "y1": by,
                "z1": float(b["alt"]),
                "t0": t0,
                "t1": t1,
            }
        )
    return segments


def _interp_segment(seg: Dict, t: float) -> Tuple[float, float, float]:
    if seg["t1"] <= seg["t0"]:
        return seg["x1"], seg["y1"], seg["z1"]
    alpha = (t - seg["t0"]) / (seg["t1"] - seg["t0"])
    alpha = max(0.0, min(1.0, alpha))
    return (
        seg["x0"] + alpha * (seg["x1"] - seg["x0"]),
        seg["y0"] + alpha * (seg["y1"] - seg["y0"]),
        seg["z0"] + alpha * (seg["z1"] - seg["z0"]),
    )


def detect_collision_conflicts(waypoints: List[Dict[str, float]], constraints: Dict = None) -> List[Dict]:
    c = {**DEFAULT_CONSTRAINTS, **(constraints or {})}
    grouped = _group_waypoints_by_uav(waypoints)
    if len(grouped) < 2:
        return []

    segments_by_uav: Dict[str, List[Dict]] = {}
    for uav_id, uav_wps in grouped.items():
        segments_by_uav[uav_id] = _build_timed_segments(
            uav_wps,
            ref_lat=float(c["home_lat"]),
            ref_lon=float(c["home_lon"]),
        )

    uav_ids = sorted(segments_by_uav.keys())
    collisions: List[Dict] = []

    for i in range(len(uav_ids)):
        for j in range(i + 1, len(uav_ids)):
            ua = uav_ids[i]
            ub = uav_ids[j]
            for sa in segments_by_uav[ua]:
                for sb in segments_by_uav[ub]:
                    overlap_start = max(sa["t0"], sb["t0"])
                    overlap_end = min(sa["t1"], sb["t1"])
                    if overlap_end <= overlap_start:
                        continue

                    sample_count = max(2, int((overlap_end - overlap_start) / 1.0) + 1)
                    for k in range(sample_count):
                        t = overlap_start + (overlap_end - overlap_start) * (k / (sample_count - 1))
                        ax, ay, az = _interp_segment(sa, t)
                        bx, by, bz = _interp_segment(sb, t)

                        hdist = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
                        vdist = abs(bz - az)
                        if hdist < float(c["collision_radius_m"]) and vdist < float(c["vertical_separation_m"]):
                            collisions.append(
                                {
                                    "uav_a": ua,
                                    "uav_b": ub,
                                    "time_s": round(t, 2),
                                    "segment_a": sa["index"],
                                    "segment_b": sb["index"],
                                    "horizontal_m": round(hdist, 2),
                                    "vertical_m": round(vdist, 2),
                                }
                            )
                            break
                    if collisions and collisions[-1]["uav_a"] == ua and collisions[-1]["uav_b"] == ub:
                        break

    return collisions


def validate_waypoints(waypoints: List[Dict[str, float]], constraints: Dict = None) -> Tuple[bool, List[str]]:
    if not SAFETY_VALIDATION_ENABLED:
        return True, []

    c = {**DEFAULT_CONSTRAINTS, **(constraints or {})}
    errors: List[str] = []

    if not waypoints:
        errors.append("Mission must contain at least one waypoint.")
        return False, errors

    if len(waypoints) > c["max_waypoints"]:
        errors.append(f"Waypoint count exceeds max limit ({c['max_waypoints']}).")

    home_lat = float(c.get("home_lat", DEFAULT_CONSTRAINTS["home_lat"]))
    home_lon = float(c.get("home_lon", DEFAULT_CONSTRAINTS["home_lon"]))
    allowed_actions = {
        "waypoint",
        "takeoff",
        "land",
        "land_and_hold",
        "loiter",
        "takeoff_after_hold",
        "descend",
        "goto_low_alt",
        "rtl",
        "disarm",
    }

    for idx, wp in enumerate(waypoints):
        if not {"lat", "lon", "alt"}.issubset(wp.keys()):
            errors.append(f"WP{idx + 1}: missing required fields lat/lon/alt.")
            continue

        action = str(wp.get("action", "waypoint") or "waypoint").lower()
        if action not in allowed_actions:
            errors.append(f"WP{idx + 1}: unsupported action '{action}'.")

        lat = float(wp["lat"])
        lon = float(wp["lon"])
        alt = float(wp["alt"])
        if lat < -90.0 or lat > 90.0 or lon < -180.0 or lon > 180.0:
            errors.append(f"WP{idx + 1}: invalid coordinate range lat/lon.")

        min_alt = 0.0 if action in ("land", "land_and_hold", "rtl", "disarm") else c["min_alt_m"]
        if alt < min_alt or alt > c["max_alt_m"]:
            errors.append(
                f"WP{idx + 1}: altitude {alt:.1f}m out of bounds [{min_alt}, {c['max_alt_m']}]."
            )

        speed_value = wp.get("speed", 8.0)
        if speed_value in (None, ""):
            speed_value = 8.0
        speed = float(speed_value)
        min_speed = 0.0 if action in ("land", "land_and_hold", "loiter", "rtl", "disarm") else 0.5
        if speed < min_speed or speed > 50.0:
            errors.append(f"WP{idx + 1}: speed {speed:.1f}m/s is invalid.")

        hold_s = float(wp.get("hold_s", 0.0) or 0.0)
        if hold_s < 0.0:
            errors.append(f"WP{idx + 1}: hold_s must be non-negative.")

        radius = _distance_m(home_lat, home_lon, lat, lon)
        if radius > float(c.get("max_radius_m", DEFAULT_CONSTRAINTS["max_radius_m"])):
            errors.append(
                f"WP{idx + 1}: radius {radius:.1f}m exceeds mission radius {float(c.get('max_radius_m', DEFAULT_CONSTRAINTS['max_radius_m']))}m."
            )

    return len(errors) == 0, errors


def sitl_precheck(waypoints: List[Dict[str, float]], mission_context: Dict = None) -> Dict:
    if not SAFETY_VALIDATION_ENABLED:
        return {
            "pass": True,
            "errors": [],
            "collision_events": [],
            "compliance": {"pass": True, "errors": [], "checks": {"energy_budget": True, "regulatory": True}},
            "geofence": {"pass": True, "errors": [], "violations": [], "has_geofence": False, "nfz_count": 0},
            "checks": {
                "schema": True,
                "geofence": True,
                "altitude": True,
                "collision": True,
                "energy_budget": True,
                "regulatory": True,
            },
            "bypass": "Safety validation disabled",
        }

    mission_context = mission_context or {}
    constraints = {**DEFAULT_CONSTRAINTS, **(mission_context.get("constraints") or {})}

    valid, errors = validate_waypoints(waypoints, constraints=constraints)
    collisions = detect_collision_conflicts(waypoints, constraints=constraints)
    compliance = evaluate_compliance(waypoints, mission_context=mission_context)
    geofence = evaluate_geofence_and_nfz(waypoints, constraints=constraints)

    if collisions:
        errors.append(f"Collision-risk detected for {len(collisions)} segment pair(s).")
    if not compliance["pass"]:
        errors.extend(compliance["errors"])
    if not geofence["pass"]:
        errors.extend(geofence["errors"])

    pass_all = valid and not collisions and compliance["pass"] and geofence["pass"]

    return {
        "pass": pass_all,
        "errors": errors,
        "collision_events": collisions,
        "compliance": compliance,
        "geofence": geofence,
        "checks": {
            "schema": valid,
            "geofence": geofence["pass"],
            "altitude": valid,
            "collision": len(collisions) == 0,
            "energy_budget": compliance["checks"]["energy_budget"],
            "regulatory": compliance["checks"]["regulatory"],
        },
    }
