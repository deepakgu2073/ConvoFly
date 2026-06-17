from typing import Dict, List

from .safety import sitl_precheck


class MissionService:
    def __init__(self, telemetry_state, runtime_state):
        self.telemetry_state = telemetry_state
        self.runtime_state = runtime_state

    def _reset_progress_state(self):
        self.telemetry_state.current_wp = 0
        self.telemetry_state.wp_action_state = "idle"
        self.telemetry_state.wp_hold_start_sim_time = None

    def _set_active_waypoint_target(self):
        if not self.telemetry_state.waypoints:
            return

        if self.telemetry_state.current_wp >= len(self.telemetry_state.waypoints):
            self.telemetry_state.current_wp = 0

        wp = self.telemetry_state.waypoints[self.telemetry_state.current_wp]
        self.runtime_state.sim_target.update({
            "lat": wp["lat"],
            "lon": wp["lon"],
            "alt": wp["alt"],
        })

    def upload(self, waypoints: List[Dict[str, float]], mission_context: Dict = None) -> Dict:
        mission_context = dict(mission_context or {})
        constraints = dict(mission_context.get("constraints") or {})
        constraints.setdefault("home_lat", float(self.telemetry_state.lat))
        constraints.setdefault("home_lon", float(self.telemetry_state.lon))
        mission_context["constraints"] = constraints
        mission_context.setdefault("remote_id_enabled", True)
        mission_context.setdefault("operator_approved", True)

        precheck = sitl_precheck(waypoints, mission_context=mission_context)
        if not precheck["pass"]:
            return {
                "ok": False,
                "message": "Mission rejected by safety validator.",
                "precheck": precheck,
            }

        self.telemetry_state.waypoints = waypoints
        self._reset_progress_state()
        return {
            "ok": True,
            "message": f"Mission uploaded: {len(waypoints)} waypoints",
            "precheck": precheck,
        }

    def clear(self) -> Dict:
        self.telemetry_state.waypoints = []
        self._reset_progress_state()
        self.runtime_state.novice_auto_pending = False
        return {"ok": True, "message": "Mission cleared"}

    def start_novice_sequence(self, takeoff_alt: float = 20.0) -> Dict:
        """
        Automatic sequence for novice users:
        1. Arm the drone
        2. Takeoff to specified altitude
        3. Switch to AUTO mode and start mission
        """
        if not self.telemetry_state.waypoints:
            return {"ok": False, "message": "No mission uploaded"}

        self._reset_progress_state()

        # Step 1: Arm
        self.telemetry_state.armed = True
        self.telemetry_state.mode = "GUIDED"

        # Step 2: Takeoff - set target altitude
        # Extract takeoff altitude from first waypoint or use default
        first_wp = self.telemetry_state.waypoints[0]
        takeoff_altitude = first_wp.get("alt", takeoff_alt)
        takeoff_altitude = max(5.0, min(120.0, takeoff_altitude))  # Clamp between 5-120m

        self.runtime_state.sim_target.update({
            "lat": self.telemetry_state.lat,
            "lon": self.telemetry_state.lon,
            "alt": takeoff_altitude,
        })

        # Step 3: Set pending flag to auto-switch to AUTO after reaching altitude
        self.runtime_state.novice_auto_pending = True
        self.telemetry_state.current_wp = 0

        return {
            "ok": True,
            "message": f"Arming, taking off to {takeoff_altitude}m, then starting mission",
            "sequence": "arm_takeoff_auto"
        }

    def start(self) -> Dict:
        self._reset_progress_state()
        self.runtime_state.novice_auto_pending = False
        self.telemetry_state.mode = "AUTO"
        self._set_active_waypoint_target()
        return {"ok": True, "message": "Mission started (AUTO mode)"}
