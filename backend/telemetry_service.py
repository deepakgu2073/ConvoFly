import math
import random
import time
from typing import Callable, Dict, Optional


class TelemetryService:
    def __init__(self, telemetry_state, runtime_state):
        self.state = telemetry_state
        self.runtime = runtime_state

    def step_sim(self, t: float, emit_status: Callable[[Dict], None]):
        if not self.state.armed:
            return

        if self.runtime.flight_start_time is None:
            self.runtime.flight_start_time = time.time()
        self.state.flight_time = int(time.time() - self.runtime.flight_start_time)

        self.state.roll = 2.5 * math.sin(t * 0.7)
        self.state.pitch = 1.8 * math.sin(t * 0.5 + 0.3)
        self.state.yaw = self.state.heading
        self.state.battery = max(0.0, self.state.battery - 0.002)

        dlat = self.runtime.sim_target["lat"] - self.state.lat
        dlon = self.runtime.sim_target["lon"] - self.state.lon
        dist = math.sqrt(dlat ** 2 + dlon ** 2)

        if dist > 0.00001:
            speed = min(0.00005, dist * 0.1)
            self.state.lat += (dlat / dist) * speed
            self.state.lon += (dlon / dist) * speed
            self.state.speed = speed * 111320
            self.state.heading = (math.degrees(math.atan2(dlon, dlat)) + 360) % 360

        if self.state.alt < self.runtime.sim_target["alt"]:
            self.state.alt = min(self.runtime.sim_target["alt"], self.state.alt + 0.5)
        elif self.state.alt > self.runtime.sim_target["alt"]:
            self.state.alt = max(self.runtime.sim_target["alt"], self.state.alt - 0.3)

        self.state.lat += random.gauss(0, 0.000002)
        self.state.lon += random.gauss(0, 0.000002)
        self.state.rssi = -65 + random.randint(-5, 5)

        # Auto-switch to AUTO mode for novice users once takeoff altitude is reached
        if self.runtime.novice_auto_pending and self.state.mode == "GUIDED":
            alt_err = abs(self.state.alt - self.runtime.sim_target["alt"])
            if alt_err < 2.0:  # Within 2m of target altitude
                self.state.mode = "AUTO"
                self.runtime.novice_auto_pending = False
                # Update sim target to first waypoint
                if self.state.waypoints:
                    wp = self.state.waypoints[0]
                    self.runtime.sim_target.update({
                        "lat": wp["lat"],
                        "lon": wp["lon"],
                        "alt": wp["alt"],
                    })
                    emit_status({
                        "msg": f"Reached takeoff altitude {self.runtime.sim_target['alt']:.0f}m, switching to AUTO, starting mission",
                        "type": "success",
                    })


        self._advance_auto_mission(t, emit_status)

    def _advance_auto_mission(self, t: float, emit_status: Callable[[Dict], None]):
        if self.state.mode != "AUTO" or not self.state.waypoints:
            return

        idx = self.state.current_wp
        if not (0 <= idx < len(self.state.waypoints)):
            return

        wp = self.state.waypoints[idx]
        action = wp.get("action", "goto")
        dlat_wp = wp["lat"] - self.state.lat
        dlon_wp = wp["lon"] - self.state.lon
        hdist_m = math.sqrt(dlat_wp ** 2 + dlon_wp ** 2) * 111320
        alt_err_m = abs((wp["alt"] or 0.0) - self.state.alt)

        if action in ("land", "land_and_hold"):
            if self.state.alt > 0.5:
                self.runtime.sim_target["alt"] = 0.0
                if self.state.wp_action_state != "landing":
                    self.state.wp_action_state = "landing"
                    emit_status({
                        "msg": f"WP{idx + 1}: Landing at ({wp['lat']:.5f}, {wp['lon']:.5f})",
                        "type": "info",
                    })
            else:
                hold_s = wp.get("hold_s", 0.0)
                if hold_s > 0:
                    if self.state.wp_action_state != "holding":
                        self.state.wp_action_state = "holding"
                        self.state.wp_hold_start_sim_time = t
                        emit_status({
                            "msg": f"WP{idx + 1}: Landed. Holding for {hold_s}s",
                            "type": "info",
                        })
                    else:
                        elapsed = t - (self.state.wp_hold_start_sim_time or t)
                        if elapsed >= hold_s:
                            self.state.wp_action_state = "idle"
                            self.state.wp_hold_start_sim_time = None
                            self.state.current_wp += 1
                            if self.state.current_wp < len(self.state.waypoints):
                                next_wp = self.state.waypoints[self.state.current_wp]
                                self.runtime.sim_target.update({
                                    "lat": next_wp["lat"],
                                    "lon": next_wp["lon"],
                                    "alt": next_wp["alt"],
                                })
                                emit_status({
                                    "msg": f"-> Hold complete, proceeding to WP{self.state.current_wp + 1}",
                                    "type": "info",
                                })
                            else:
                                self.state.current_wp = len(self.state.waypoints)
                                self.state.mode = "LAND"
                                self.runtime.sim_target["alt"] = 0.0
                                emit_status({"msg": "Mission complete, landing", "type": "success"})
                        else:
                            remaining = hold_s - elapsed
                            if remaining > 0.1:
                                emit_status({
                                    "msg": f"WP{idx + 1}: Holding... {remaining:.1f}s remaining",
                                    "type": "info",
                                })
                else:
                    self.state.wp_action_state = "idle"
                    self.state.current_wp += 1
                    if self.state.current_wp < len(self.state.waypoints):
                        next_wp = self.state.waypoints[self.state.current_wp]
                        self.runtime.sim_target.update({
                            "lat": next_wp["lat"],
                            "lon": next_wp["lon"],
                            "alt": next_wp["alt"],
                        })
                        emit_status({
                            "msg": f"-> Landed at WP{idx + 1}, proceeding to WP{self.state.current_wp + 1}",
                            "type": "info",
                        })
                    else:
                        self.state.current_wp = len(self.state.waypoints)
                        self.state.mode = "LAND"
                        self.runtime.sim_target["alt"] = 0.0
                        emit_status({"msg": "Mission complete, landing", "type": "success"})

        elif action == "loiter":
            hold_s = float(wp.get("hold_s", 5.0) or 5.0)
            if self.state.wp_action_state != "loitering":
                self.state.wp_action_state = "loitering"
                self.state.wp_hold_start_sim_time = t
                emit_status({"msg": f"WP{idx + 1}: Loitering for {hold_s}s", "type": "info"})
            else:
                elapsed = t - (self.state.wp_hold_start_sim_time or t)
                if elapsed >= hold_s:
                    self.state.wp_action_state = "idle"
                    self.state.wp_hold_start_sim_time = None
                    self.state.current_wp += 1
                    if self.state.current_wp < len(self.state.waypoints):
                        next_wp = self.state.waypoints[self.state.current_wp]
                        self.runtime.sim_target.update({
                            "lat": next_wp["lat"],
                            "lon": next_wp["lon"],
                            "alt": next_wp["alt"],
                        })
                        emit_status({"msg": f"-> Loiter complete, proceeding to WP{self.state.current_wp + 1}", "type": "info"})
                    else:
                        self.state.current_wp = len(self.state.waypoints)
                        self.state.mode = "AUTO"
                        emit_status({"msg": "Mission complete", "type": "success"})

        elif action == "rtl":
            if self.state.wp_action_state != "returning_home":
                self.state.wp_action_state = "returning_home"
                home_lat = getattr(self.runtime, "sitl_home_lat", 28.6139)
                home_lon = getattr(self.runtime, "sitl_home_lon", 77.2090)
                self.runtime.sim_target.update({"lat": home_lat, "lon": home_lon, "alt": max(10.0, float(wp.get("alt", 20.0) or 20.0))})
                emit_status({"msg": f"WP{idx + 1}: Returning to launch", "type": "info"})
            elif hdist_m < 10.0:
                self.state.wp_action_state = "idle"
                self.state.current_wp += 1
                if self.state.current_wp < len(self.state.waypoints):
                    next_wp = self.state.waypoints[self.state.current_wp]
                    self.runtime.sim_target.update({"lat": next_wp["lat"], "lon": next_wp["lon"], "alt": next_wp["alt"]})
                else:
                    self.state.current_wp = len(self.state.waypoints)
                    self.state.mode = "RTL"
                    emit_status({"msg": "Mission complete, RTL reached home", "type": "success"})

        elif action == "disarm":
            self.state.armed = False
            self.state.mode = "STABILIZE"
            self.state.wp_action_state = "idle"
            self.state.current_wp = min(self.state.current_wp + 1, len(self.state.waypoints))
            emit_status({"msg": f"WP{idx + 1}: Disarmed", "type": "warning"})

        elif action in ("takeoff", "takeoff_after_hold"):
            if self.state.wp_action_state != "taking_off":
                self.state.wp_action_state = "taking_off"
                self.runtime.sim_target.update({
                    "lat": wp["lat"],
                    "lon": wp["lon"],
                    "alt": wp["alt"],
                })
                emit_status({
                    "msg": f"WP{idx + 1}: Taking off to {wp['alt']}m",
                    "type": "info",
                })
            elif hdist_m < 4.0 and alt_err_m < 2.0:
                self.state.wp_action_state = "idle"
                self.state.current_wp += 1
                if self.state.current_wp < len(self.state.waypoints):
                    next_wp = self.state.waypoints[self.state.current_wp]
                    self.runtime.sim_target.update({
                        "lat": next_wp["lat"],
                        "lon": next_wp["lon"],
                        "alt": next_wp["alt"],
                    })
                    emit_status({
                        "msg": f"-> Took off from WP{idx + 1}, proceeding to WP{self.state.current_wp + 1}",
                        "type": "info",
                    })
                else:
                    self.state.current_wp = len(self.state.waypoints)
                    self.state.mode = "LAND"
                    self.runtime.sim_target["alt"] = 0.0
                    emit_status({"msg": "Mission complete, landing", "type": "success"})

        elif action in ("descend", "goto_low_alt"):
            if self.state.wp_action_state != "descending":
                self.state.wp_action_state = "descending"
                self.runtime.sim_target.update({
                    "lat": wp["lat"],
                    "lon": wp["lon"],
                    "alt": wp["alt"],
                })
                emit_status({
                    "msg": f"WP{idx + 1}: Descending to {wp['alt']}m at ({wp['lat']:.5f}, {wp['lon']:.5f})",
                    "type": "info",
                })
            elif hdist_m < 4.0 and alt_err_m < 2.0:
                self.state.wp_action_state = "idle"
                self.state.current_wp += 1
                if self.state.current_wp < len(self.state.waypoints):
                    next_wp = self.state.waypoints[self.state.current_wp]
                    self.runtime.sim_target.update({
                        "lat": next_wp["lat"],
                        "lon": next_wp["lon"],
                        "alt": next_wp["alt"],
                    })
                    emit_status({
                        "msg": f"-> Reached WP{idx + 1}, proceeding to WP{self.state.current_wp + 1}",
                        "type": "info",
                    })
                else:
                    self.state.current_wp = len(self.state.waypoints)
                    self.state.mode = "LAND"
                    self.runtime.sim_target["alt"] = 0.0
                    emit_status({"msg": "Mission complete, landing", "type": "success"})

        else:
            if self.state.wp_action_state != "moving":
                self.state.wp_action_state = "moving"
                self.runtime.sim_target.update({
                    "lat": wp["lat"],
                    "lon": wp["lon"],
                    "alt": wp["alt"],
                })
                emit_status({
                    "msg": f"WP{idx + 1}: Moving to ({wp['lat']:.5f}, {wp['lon']:.5f}) @ {wp['alt']}m",
                    "type": "info",
                })
            elif hdist_m < 4.0 and alt_err_m < 2.0:
                self.state.wp_action_state = "idle"
                self.state.current_wp += 1
                if self.state.current_wp < len(self.state.waypoints):
                    next_wp = self.state.waypoints[self.state.current_wp]
                    self.runtime.sim_target.update({
                        "lat": next_wp["lat"],
                        "lon": next_wp["lon"],
                        "alt": next_wp["alt"],
                    })
                    emit_status({
                        "msg": f"-> Reached WP{idx + 1}, proceeding to WP{self.state.current_wp + 1}",
                        "type": "info",
                    })
                else:
                    self.state.current_wp = len(self.state.waypoints)
                    self.state.mode = "LAND"
                    self.runtime.sim_target["alt"] = 0.0
                    emit_status({"msg": "Mission complete, landing", "type": "success"})

    def payload(self) -> Dict:
        return {
            "lat": self.state.lat,
            "lon": self.state.lon,
            "alt": self.state.alt,
            "heading": self.state.heading,
            "speed": self.state.speed,
            "battery": self.state.battery,
            "mode": self.state.mode,
            "armed": self.state.armed,
            "gps_fix": self.state.gps_fix,
            "satellites": self.state.satellites,
            "roll": self.state.roll,
            "pitch": self.state.pitch,
            "yaw": self.state.yaw,
            "vx": self.state.vx,
            "vy": self.state.vy,
            "vz": self.state.vz,
            "connected": self.state.connected,
            "flight_time": self.state.flight_time,
            "rssi": self.state.rssi,
        }
