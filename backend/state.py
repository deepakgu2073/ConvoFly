from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class TelemetryState:
    lat: float = 28.6139
    lon: float = 77.2090
    alt: float = 0.0
    heading: float = 0.0
    speed: float = 0.0
    battery: float = 87.0
    mode: str = "STABILIZE"
    armed: bool = False
    gps_fix: int = 3
    satellites: int = 12
    rssi: int = -65
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    connected: bool = False
    flight_time: int = 0
    waypoints: List[Dict[str, float]] = field(default_factory=list)
    current_wp: int = 0
    wp_action_state: str = "idle"
    wp_hold_start_sim_time: Optional[float] = None


@dataclass
class RuntimeState:
    running: bool = False
    flight_start_time: Optional[float] = None
    connection_string: Optional[str] = None
    sitl_exe_path: Optional[str] = None
    sitl_home_lat: float = 28.6139
    sitl_home_lon: float = 77.2090
    sitl_home_alt: float = 185.0
    sim_target: Dict[str, float] = field(default_factory=lambda: {
        "lat": 28.6139,
        "lon": 77.2090,
        "alt": 0.0,
    })
    novice_auto_pending: bool = False  # True when waiting to switch to AUTO after takeoff


@dataclass
class ExperimentRecord:
    timestamp: str
    event: str
    operator_type: str
    interface_variant: str
    payload: Dict
