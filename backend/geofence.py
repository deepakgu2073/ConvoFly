from typing import Dict, List, Tuple


def _point_in_polygon(lat: float, lon: float, polygon: List[Dict[str, float]]) -> bool:
    if len(polygon) < 3:
        return False

    inside = False
    x = lon
    y = lat
    j = len(polygon) - 1

    for i in range(len(polygon)):
        xi = float(polygon[i]["lon"])
        yi = float(polygon[i]["lat"])
        xj = float(polygon[j]["lon"])
        yj = float(polygon[j]["lat"])

        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i

    return inside


def _orientation(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
    return (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])


def _on_segment(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> bool:
    return (
        min(a[0], c[0]) <= b[0] <= max(a[0], c[0])
        and min(a[1], c[1]) <= b[1] <= max(a[1], c[1])
    )


def _segments_intersect(p1: Tuple[float, float], q1: Tuple[float, float], p2: Tuple[float, float], q2: Tuple[float, float]) -> bool:
    o1 = _orientation(p1, q1, p2)
    o2 = _orientation(p1, q1, q2)
    o3 = _orientation(p2, q2, p1)
    o4 = _orientation(p2, q2, q1)

    if (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0):
        return True

    eps = 1e-12
    if abs(o1) < eps and _on_segment(p1, p2, q1):
        return True
    if abs(o2) < eps and _on_segment(p1, q2, q1):
        return True
    if abs(o3) < eps and _on_segment(p2, p1, q2):
        return True
    if abs(o4) < eps and _on_segment(p2, q1, q2):
        return True
    return False


def _segment_intersects_polygon(a: Dict[str, float], b: Dict[str, float], polygon: List[Dict[str, float]]) -> bool:
    if len(polygon) < 3:
        return False

    p1 = (float(a["lon"]), float(a["lat"]))
    q1 = (float(b["lon"]), float(b["lat"]))

    for i in range(len(polygon)):
        j = (i + 1) % len(polygon)
        p2 = (float(polygon[i]["lon"]), float(polygon[i]["lat"]))
        q2 = (float(polygon[j]["lon"]), float(polygon[j]["lat"]))
        if _segments_intersect(p1, q1, p2, q2):
            return True

    return False


def evaluate_geofence_and_nfz(waypoints: List[Dict[str, float]], constraints: Dict) -> Dict:
    geofence = constraints.get("geofence_polygon") or []
    nfz_list = constraints.get("no_fly_zones") or []

    errors: List[str] = []
    violations: List[Dict] = []

    for idx, wp in enumerate(waypoints):
        lat = float(wp["lat"])
        lon = float(wp["lon"])

        if geofence and not _point_in_polygon(lat, lon, geofence):
            errors.append(f"WP{idx + 1}: outside mission geofence polygon.")
            violations.append({"type": "geofence", "wp": idx + 1})

        for zidx, zone in enumerate(nfz_list):
            zone_poly = zone.get("polygon", zone)
            if _point_in_polygon(lat, lon, zone_poly):
                zone_name = zone.get("name", f"NFZ-{zidx + 1}") if isinstance(zone, dict) else f"NFZ-{zidx + 1}"
                errors.append(f"WP{idx + 1}: inside no-fly zone {zone_name}.")
                violations.append({"type": "nfz", "wp": idx + 1, "zone": zone_name})

    for i in range(len(waypoints) - 1):
        a = waypoints[i]
        b = waypoints[i + 1]

        if geofence and _segment_intersects_polygon(a, b, geofence):
            # If both points are inside geofence, boundary touch is acceptable; otherwise it is a breach.
            inside_a = _point_in_polygon(float(a["lat"]), float(a["lon"]), geofence)
            inside_b = _point_in_polygon(float(b["lat"]), float(b["lon"]), geofence)
            if not (inside_a and inside_b):
                errors.append(f"Segment {i + 1}->{i + 2}: crosses geofence boundary.")
                violations.append({"type": "geofence_segment", "segment": f"{i + 1}->{i + 2}"})

        for zidx, zone in enumerate(nfz_list):
            zone_poly = zone.get("polygon", zone)
            if _segment_intersects_polygon(a, b, zone_poly):
                zone_name = zone.get("name", f"NFZ-{zidx + 1}") if isinstance(zone, dict) else f"NFZ-{zidx + 1}"
                errors.append(f"Segment {i + 1}->{i + 2}: intersects no-fly zone {zone_name}.")
                violations.append({"type": "nfz_segment", "segment": f"{i + 1}->{i + 2}", "zone": zone_name})

    return {
        "pass": len(errors) == 0,
        "errors": errors,
        "violations": violations,
        "has_geofence": len(geofence) >= 3,
        "nfz_count": len(nfz_list),
    }
