import math
from typing import Dict, List


DEFAULT_COMPLIANCE_POLICY = {
    "max_alt_m": 120.0,
    "max_speed_mps": 20.0,
    "required_battery_reserve_pct": 20.0,
    "battery_capacity_wh": 220.0,
    "base_energy_wh_per_km": 22.0,
    "remote_id_required": True,
    "operator_override_required": True,
}


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compute great-circle distance using Haversine formula (meters)."""
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


def _estimate_energy_usage_wh(waypoints: List[Dict[str, float]], policy: Dict, weather: Dict) -> Dict:
    if len(waypoints) < 2:
        return {"distance_m": 0.0, "mean_speed_mps": 0.0, "wind_speed_mps": 0.0, "estimated_wh": 0.0}

    total_distance_m = 0.0
    speeds: List[float] = []
    for i in range(len(waypoints) - 1):
        a = waypoints[i]
        b = waypoints[i + 1]
        total_distance_m += _distance_m(float(a["lat"]), float(a["lon"]), float(b["lat"]), float(b["lon"]))
        speeds.append(float(a.get("speed", 8.0) or 8.0))

    mean_speed = sum(speeds) / len(speeds) if speeds else 8.0
    mean_speed = max(0.5, mean_speed)
    wind_speed = float(weather.get("wind_speed_mps", 0.0) or 0.0)

    # Headwind factor increases consumption; tailwind uncertainty is not credited for safety.
    wind_factor = 1.0 + max(0.0, min(1.0, wind_speed / mean_speed)) * 0.6

    base_wh_per_km = float(policy["base_energy_wh_per_km"])
    estimated_wh = (total_distance_m / 1000.0) * base_wh_per_km * wind_factor

    return {
        "distance_m": total_distance_m,
        "mean_speed_mps": mean_speed,
        "wind_speed_mps": wind_speed,
        "estimated_wh": estimated_wh,
    }


def evaluate_compliance(waypoints: List[Dict[str, float]], mission_context: Dict = None) -> Dict:
    mission_context = mission_context or {}
    p = mission_context.get("policy", DEFAULT_COMPLIANCE_POLICY)
    weather = mission_context.get("weather", {})

    errors: List[str] = []

    for i, wp in enumerate(waypoints):
        alt = float(wp.get("alt", 0.0))
        speed = float(wp.get("speed", 8.0) or 8.0)

        if alt > float(p["max_alt_m"]):
            errors.append(f"WP{i + 1}: exceeds regulatory altitude cap of {p['max_alt_m']}m.")
        if speed > float(p["max_speed_mps"]):
            errors.append(f"WP{i + 1}: exceeds policy speed cap of {p['max_speed_mps']}m/s.")

    energy = _estimate_energy_usage_wh(waypoints, p, weather)
    reserve_fraction = float(p["required_battery_reserve_pct"]) / 100.0
    usable_energy = float(p["battery_capacity_wh"]) * max(0.0, 1.0 - reserve_fraction)
    energy_risk = energy["estimated_wh"] > usable_energy
    if energy_risk:
        errors.append(
            "Estimated mission energy exceeds available budget after reserve policy "
            f"({energy['estimated_wh']:.1f}Wh > {usable_energy:.1f}Wh)."
        )

    if bool(p["remote_id_required"]) and not bool(mission_context.get("remote_id_enabled", False)):
        errors.append("Remote ID is required by policy but not enabled in mission context.")

    if bool(p["operator_override_required"]) and not bool(mission_context.get("operator_approved", False)):
        errors.append("Operator approval is required by policy before mission execution.")

    checks = {
        "remote_id": bool(mission_context.get("remote_id_enabled", False)) if p["remote_id_required"] else True,
        "operator_override": bool(mission_context.get("operator_approved", False)) if p["operator_override_required"] else True,
        "energy_budget": not energy_risk,
        "regulatory": len(errors) == 0,
    }

    return {
        "pass": len(errors) == 0,
        "errors": errors,
        "energy": {
            "distance_m": round(energy["distance_m"], 2),
            "mean_speed_mps": round(energy["mean_speed_mps"], 2),
            "wind_speed_mps": round(energy["wind_speed_mps"], 2),
            "estimated_wh": round(energy["estimated_wh"], 2),
            "usable_wh": round(usable_energy, 2),
        },
        "checks": checks,
    }
