"""
Ground Control Station backend.

This module wires Flask-SocketIO routes to modular services:
- mission service
- safety validator
- telemetry simulation
- experiment logger
- LLM planning pipeline
"""

import csv
import json
import math
import os
import sys
import subprocess
import threading
import time
import re
from pathlib import Path
from typing import Callable, Optional

import requests
from flask import Flask, jsonify, render_template
from flask_socketio import SocketIO, emit
from dotenv import load_dotenv

from backend.experiment import ExperimentLogger
from backend.llm_pipeline import LLMMissionPipeline
from backend.mission_service import MissionService
from backend.state import RuntimeState, TelemetryState
from backend.telemetry_service import TelemetryService

# Add scripts to path for metric analysis
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'scripts'))

# Load .env from project root if present
load_dotenv()


# Optional DroneKit support (falls back to simulation mode).
try:
    from dronekit import Command, LocationGlobalRelative, VehicleMode, connect
    from pymavlink import mavutil

    DRONEKIT_AVAILABLE = True
except ImportError:
    DRONEKIT_AVAILABLE = False


app = Flask(__name__, template_folder='templates', static_folder='static')
# Load secret key from environment when available
app.config['SECRET_KEY'] = os.getenv('SECRET_KEY', 'gcs_secret_2024')
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

# Configuration from environment variables
MAX_RETRIES = int(os.getenv('GCS_LLM_MAX_RETRIES', '2'))

telemetry_state = TelemetryState()
runtime_state = RuntimeState()
telemetry_service = TelemetryService(telemetry_state, runtime_state)
mission_service = MissionService(telemetry_state, runtime_state)
llm_pipeline = LLMMissionPipeline(max_retries=MAX_RETRIES)
experiment_logger = ExperimentLogger(log_dir='logs')

vehicle = None
telemetry_thread = None
novice_mode = True  # Default to novice mode for simplified mission sequence
sitl_process = None
DEFAULT_SITL_EXE_PATH = os.getenv(
    'GCS_SITL_EXE_PATH',
    r'C:\Users\hp\OneDrive\Documents\Mission Planner\sitl\ArduCopter.exe',
)


def _resolve_sitl_exe_path(exe_path: Optional[str] = None) -> str:
    candidate = (exe_path or runtime_state.sitl_exe_path or DEFAULT_SITL_EXE_PATH or '').strip()
    return os.path.expandvars(os.path.expanduser(candidate))


def _launch_sitl_process(exe_path: str, home_lat: float, home_lon: float, home_alt: float):
    resolved = _resolve_sitl_exe_path(exe_path)
    if not resolved:
        raise FileNotFoundError('SITL executable path is empty')

    exe = Path(resolved)
    if not exe.exists():
        raise FileNotFoundError(f'SITL executable not found: {resolved}')

    args = [
        str(exe),
        '--home', f'{home_lat},{home_lon},{home_alt},0',
        '--model', 'quad',
    ]
    creationflags = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
    return subprocess.Popen(
        args,
        cwd=str(exe.parent),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        creationflags=creationflags,
    )


def _emit_status(payload):
    socketio.emit('status', payload)


def _meters_to_latlon_offsets(center_lat: float, center_lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    lat_scale = 111_320.0
    lon_scale = max(1e-6, 111_320.0 * math.cos(math.radians(center_lat)))
    return center_lat + (north_m / lat_scale), center_lon + (east_m / lon_scale)


def _extract_first_number(text: str, patterns: list[str], default: float) -> float:
    for pattern in patterns:
        match = re.search(pattern, text, re.IGNORECASE)
        if match:
            try:
                return float(match.group(1))
            except Exception:
                continue
    return default


def _extract_first_int(text: str, patterns: list[str], default: int) -> int:
    return int(round(_extract_first_number(text, patterns, float(default))))


def _extract_duration_seconds(text: str, default: float = 10.0) -> float:
    seconds = _extract_first_number(
        text,
        [
            r'(?:hold|wait|loiter|after)\s*(\d+(?:\.\d+)?)\s*(?:s|sec|secs|second|seconds)',
            r'(\d+(?:\.\d+)?)\s*(?:s|sec|secs|second|seconds)',
        ],
        default,
    )
    return max(0.0, min(600.0, seconds))


def _mock_llm_call(user_request: str) -> str:
    """Deterministic local generator used when no external LLM provider is wired."""
    request = (user_request or '').strip().lower()
    center_lat = float(getattr(telemetry_state, 'lat', 28.6139) or 28.6139)
    center_lon = float(getattr(telemetry_state, 'lon', 77.2090) or 77.2090)

    altitude = _extract_first_number(
        request,
        [r'(?:altitude|at|around|fly at)\s*(\d+(?:\.\d+)?)\s*m', r'(\d+(?:\.\d+)?)\s*m'],
        40.0,
    )
    altitude = max(5.0, min(120.0, altitude))

    speed = _extract_first_number(
        request,
        [r'(?:speed|at)\s*(\d+(?:\.\d+)?)\s*m/s', r'(\d+(?:\.\d+)?)\s*mps'],
        6.0,
    )
    speed = max(0.5, min(20.0, speed))

    point_count = _extract_first_int(
        request,
        [r'(?:with|for|using)\s*(\d+)\s*(?:points|waypoints|wps)', r'(?:\b)(\d+)\s*(?:point|points|waypoints|wps)'],
        4,
    )
    point_count = max(1, min(12, point_count))

    radius_m = _extract_first_number(
        request,
        [r'(?:radius|around|orbit)\s*(\d+(?:\.\d+)?)\s*m', r'(\d+(?:\.\d+)?)\s*m\s*radius'],
        120.0,
    )
    radius_m = max(25.0, min(1000.0, radius_m))

    mission_type = 'custom_mission'
    waypoints = []

    is_complex_sequence = any(keyword in request for keyword in (
        'then',
        'after',
        'land',
        'take off',
        'takeoff',
        'descend',
        'ascend',
        'multi',
        'phase',
    ))

    if is_complex_sequence:
        mission_type = 'multi_phase_ops'
        hold_s = _extract_duration_seconds(request, default=10.0)
        cruise_alt = altitude
        approach_alt = max(5.0, min(cruise_alt, 15.0))

        point_a_lat, point_a_lon = _meters_to_latlon_offsets(center_lat, center_lon, radius_m * 0.8, radius_m * 0.2)
        point_b_lat, point_b_lon = _meters_to_latlon_offsets(center_lat, center_lon, radius_m * 1.2, -radius_m * 0.7)
        point_c_lat, point_c_lon = _meters_to_latlon_offsets(center_lat, center_lon, -radius_m * 0.9, radius_m * 1.1)

        waypoints = [
            {'lat': round(center_lat, 6), 'lon': round(center_lon, 6), 'alt': round(cruise_alt, 1), 'speed': round(speed, 1), 'action': 'takeoff'},
            {'lat': round(point_a_lat, 6), 'lon': round(point_a_lon, 6), 'alt': round(cruise_alt, 1), 'speed': round(speed, 1), 'action': 'goto'},
            {'lat': round(point_a_lat, 6), 'lon': round(point_a_lon, 6), 'alt': round(approach_alt, 1), 'speed': round(max(0.5, speed * 0.6), 1), 'action': 'descend'},
            {'lat': round(point_a_lat, 6), 'lon': round(point_a_lon, 6), 'alt': 0.0, 'speed': 0.0, 'action': 'land', 'hold_s': round(hold_s, 1)},
            {'lat': round(point_a_lat, 6), 'lon': round(point_a_lon, 6), 'alt': round(cruise_alt, 1), 'speed': round(speed, 1), 'action': 'takeoff_after_hold'},
            {'lat': round(point_b_lat, 6), 'lon': round(point_b_lon, 6), 'alt': round(cruise_alt, 1), 'speed': round(speed, 1), 'action': 'goto'},
            {'lat': round(point_b_lat, 6), 'lon': round(point_b_lon, 6), 'alt': round(approach_alt, 1), 'speed': round(max(0.5, speed * 0.7), 1), 'action': 'descend'},
            {'lat': round(point_c_lat, 6), 'lon': round(point_c_lon, 6), 'alt': round(approach_alt, 1), 'speed': round(speed, 1), 'action': 'goto_low_alt'},
        ]

    elif any(keyword in request for keyword in ('go to', 'goto', 'point to point', 'single waypoint')):
        mission_type = 'goto'
        waypoint_lat, waypoint_lon = _meters_to_latlon_offsets(center_lat, center_lon, radius_m, radius_m * 0.35)
        waypoints = [{'lat': round(waypoint_lat, 6), 'lon': round(waypoint_lon, 6), 'alt': round(altitude, 1), 'speed': round(speed, 1)}]
    elif any(keyword in request for keyword in ('grid', 'survey', 'search', 'lawnmower')):
        mission_type = 'survey_grid'
        half = radius_m * 0.5
        rows = 3
        cols = 3
        lane_spacing = max(20.0, (half * 2) / max(1, rows - 1))
        for row in range(rows):
            east_offset = -half if row % 2 == 0 else half
            north = -half + row * lane_spacing
            line_points = []
            for col in range(cols):
                east = east_offset + (col * (half / max(1, cols - 1)) * (1 if row % 2 == 0 else -1))
                lat, lon = _meters_to_latlon_offsets(center_lat, center_lon, north, east)
                line_points.append({'lat': round(lat, 6), 'lon': round(lon, 6), 'alt': round(altitude, 1), 'speed': round(speed, 1)})
            waypoints.extend(line_points if row % 2 == 0 else list(reversed(line_points)))
    elif any(keyword in request for keyword in ('circle', 'orbit', 'patrol', 'perimeter', 'loop')):
        mission_type = 'perimeter_patrol'
        effective_count = max(4, point_count)
        for index in range(effective_count):
            angle = (2.0 * math.pi * index) / effective_count
            north = math.sin(angle) * radius_m
            east = math.cos(angle) * radius_m
            lat, lon = _meters_to_latlon_offsets(center_lat, center_lon, north, east)
            waypoints.append({'lat': round(lat, 6), 'lon': round(lon, 6), 'alt': round(altitude, 1), 'speed': round(speed, 1)})
    else:
        mission_type = 'waypoint_route'
        step = max(20.0, radius_m / max(1, point_count))
        for index in range(point_count):
            north = -radius_m * 0.5 + (step * index)
            east = radius_m * 0.3 if index % 2 == 0 else -radius_m * 0.3
            lat, lon = _meters_to_latlon_offsets(center_lat, center_lon, north, east)
            waypoints.append({'lat': round(lat, 6), 'lon': round(lon, 6), 'alt': round(altitude, 1), 'speed': round(speed, 1)})

    mission = {
        'mission_type': mission_type,
        'waypoints': waypoints,
        'constraints': {
            'max_alt_m': 120,
            'min_alt_m': 5,
            'max_waypoints': 200,
            'home_lat': round(center_lat, 6),
            'home_lon': round(center_lon, 6),
            'max_radius_m': 5000,
        },
    }
    return json.dumps(mission)


def _call_llm_provider(prompt: str, provider: str, api_key: str, model: str) -> str:
    """Call external LLM provider and return response text."""
    timeout = 30
    headers = {}
    
    if provider == 'openai':
        url = 'https://api.openai.com/v1/chat/completions'
        headers = {
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json',
        }
        payload = {
            'model': model or 'gpt-4-turbo',
            'messages': [
                {'role': 'system', 'content': 'You are a UAV mission planning expert.'},
                {'role': 'user', 'content': prompt},
            ],
            'temperature': 0.5,
        }
        try:
            response = requests.post(url, json=payload, headers=headers, timeout=timeout)
            response.raise_for_status()
            data = response.json()
            return data['choices'][0]['message']['content']
        except Exception as e:
            raise RuntimeError(f'OpenAI API call failed: {e}')
    
    elif provider == 'groq':
        url = 'https://api.groq.com/openai/v1/chat/completions'
        headers = {
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json',
        }
        payload = {
            'model': model or 'mixtral-8x7b-32768',
            'messages': [
                {'role': 'system', 'content': 'You are a UAV mission planning expert.'},
                {'role': 'user', 'content': prompt},
            ],
            'temperature': 0.5,
        }
        try:
            response = requests.post(url, json=payload, headers=headers, timeout=timeout)
            response.raise_for_status()
            data = response.json()
            return data['choices'][0]['message']['content']
        except Exception as e:
            raise RuntimeError(f'Groq API call failed: {e}')
    
    elif provider == 'github_models':
        url = 'https://models.inference.ai.azure.com/chat/completions'
        headers = {
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json',
        }
        payload = {
            'model': model or 'gpt-4-turbo',
            'messages': [
                {'role': 'system', 'content': 'You are a UAV mission planning expert.'},
                {'role': 'user', 'content': prompt},
            ],
            'temperature': 0.5,
        }
        try:
            response = requests.post(url, json=payload, headers=headers, timeout=timeout)
            response.raise_for_status()
            data = response.json()
            return data['choices'][0]['message']['content']
        except Exception as e:
            raise RuntimeError(f'GitHub Models API call failed: {e}')
    
    elif provider == 'anthropic':
        url = 'https://api.anthropic.com/v1/messages'
        headers = {
            'x-api-key': api_key,
            'anthropic-version': '2023-06-01',
            'Content-Type': 'application/json',
        }
        payload = {
            'model': model or 'claude-3-sonnet-20240229',
            'max_tokens': 2048,
            'system': 'You are a UAV mission planning expert.',
            'messages': [{'role': 'user', 'content': prompt}],
        }
        try:
            response = requests.post(url, json=payload, headers=headers, timeout=timeout)
            response.raise_for_status()
            data = response.json()
            return data['content'][0]['text']
        except Exception as e:
            raise RuntimeError(f'Anthropic API call failed: {e}')
    
    else:
        raise ValueError(f'Unsupported LLM provider: {provider}')


def _set_sim_target_from_current_wp():
    wps = telemetry_state.waypoints
    idx = telemetry_state.current_wp
    if 0 <= idx < len(wps):
        wp = wps[idx]
        runtime_state.sim_target.update({'lat': wp['lat'], 'lon': wp['lon'], 'alt': wp['alt']})


def _set_mission_start_index(cmds):
    if cmds.count > 1:
        cmds.next = 1
    elif cmds.count == 1:
        cmds.next = 0


def _get_real_telemetry_payload():
    loc = vehicle.location.global_relative_frame
    att = vehicle.attitude
    vel = vehicle.velocity
    return {
        'lat': loc.lat or 0,
        'lon': loc.lon or 0,
        'alt': round(loc.alt or 0, 1),
        'heading': vehicle.heading or 0,
        'speed': round(vehicle.groundspeed or 0, 1),
        'battery': vehicle.battery.level if vehicle.battery and vehicle.battery.level else 0,
        'mode': str(vehicle.mode.name),
        'armed': vehicle.armed,
        'gps_fix': vehicle.gps_0.fix_type,
        'satellites': vehicle.gps_0.satellites_visible,
        'roll': round(math.degrees(att.roll), 1),
        'pitch': round(math.degrees(att.pitch), 1),
        'yaw': round(math.degrees(att.yaw), 1),
        'vx': round(vel[0] if vel else 0, 2),
        'vy': round(vel[1] if vel else 0, 2),
        'vz': round(vel[2] if vel else 0, 2),
        'connected': True,
        'flight_time': telemetry_state.flight_time,
    }


def _real_telemetry_loop():
    while runtime_state.running and vehicle:
        payload = _get_real_telemetry_payload()

        # In novice mode, switch to AUTO only after reaching takeoff altitude.
        if runtime_state.novice_auto_pending and str(vehicle.mode.name) == 'GUIDED':
            target_alt = float(runtime_state.sim_target.get('alt', 0.0) or 0.0)
            if payload['alt'] >= max(2.0, target_alt - 1.5):
                cmds = vehicle.commands
                cmds.download()
                cmds.wait_ready()
                _set_mission_start_index(cmds)
                vehicle.mode = VehicleMode('AUTO')
                runtime_state.novice_auto_pending = False
                socketio.emit('status', {
                    'msg': f"Reached takeoff altitude {target_alt:.0f}m, switching to AUTO, starting mission",
                    'type': 'success',
                })

        socketio.emit('telemetry', payload)
        time.sleep(0.1)


def _sim_telemetry_loop():
    status_callback: Callable = lambda payload: socketio.emit('status', payload)
    t = 0.0
    while runtime_state.running:
        t += 0.1
        telemetry_service.step_sim(t, status_callback)
        socketio.emit('telemetry', telemetry_service.payload())
        time.sleep(0.1)


def _start_telemetry_thread():
    global telemetry_thread
    runtime_state.running = True

    if DRONEKIT_AVAILABLE and vehicle:
        telemetry_thread = threading.Thread(target=_real_telemetry_loop, daemon=True)
    else:
        telemetry_thread = threading.Thread(target=_sim_telemetry_loop, daemon=True)

    telemetry_thread.start()


@socketio.on('connect')
def on_connect():
    emit('status', {'msg': 'Connected to GCS Server', 'sim': not DRONEKIT_AVAILABLE})


@socketio.on('connect_drone')
def on_connect_drone(data):
    global vehicle
    conn = data.get('connection_string', 'tcp:127.0.0.1:5760')

    if DRONEKIT_AVAILABLE:
        try:
            emit('status', {'msg': f'Connecting to {conn}...'})
            vehicle = connect(conn, wait_ready=True, timeout=120, heartbeat_timeout=30)
            runtime_state.connection_string = conn
            telemetry_state.connected = True
            emit('status', {'msg': 'Drone connected', 'type': 'success'})
        except Exception as exc:
            msg = str(exc)
            if 'timeout' in msg.lower() or 'timed out' in msg.lower():
                msg = f'Connection timed out while initializing {conn}. Ensure SITL is running and retry.'
            emit('status', {'msg': f'Connection failed: {msg}', 'type': 'error'})
            return
    else:
        telemetry_state.connected = True
        emit('status', {'msg': 'Simulation connected (DroneKit not installed)', 'type': 'success'})

    experiment_logger.log_event('connect_drone', {'connection_string': conn})
    _start_telemetry_thread()


@socketio.on('disconnect_drone')
def on_disconnect_drone(data=None):  # Added data=None to capture the empty payload object
    # Keep your existing disconnection/cleanup logic here
    pass
    global vehicle
    runtime_state.running = False
    runtime_state.flight_start_time = None
    runtime_state.novice_auto_pending = False
    telemetry_state.connected = False
    telemetry_state.armed = False
    telemetry_state.mode = 'STABILIZE'
    telemetry_state.wp_action_state = 'idle'
    telemetry_state.wp_hold_start_sim_time = None

    if vehicle:
        vehicle.close()
        vehicle = None

    emit('status', {'msg': 'Drone disconnected', 'type': 'warning'})
    socketio.emit('armed_state', {'armed': False})
    socketio.emit('connection_state', {'connected': False})
    experiment_logger.log_event('disconnect_drone', {})


@socketio.on('set_sitl_home')
def on_set_sitl_home(data):
    data = data or {}
    lat = float(data.get('lat', runtime_state.sitl_home_lat))
    lon = float(data.get('lon', runtime_state.sitl_home_lon))
    alt = float(data.get('alt', runtime_state.sitl_home_alt))
    exe_path = data.get('exe_path')

    runtime_state.sitl_home_lat = lat
    runtime_state.sitl_home_lon = lon
    runtime_state.sitl_home_alt = alt
    if exe_path:
        runtime_state.sitl_exe_path = exe_path

    emit('status', {
        'msg': f'SITL home set to ({lat:.5f}, {lon:.5f}) @ {alt:.0f}m',
        'type': 'info',
    })
    socketio.emit('sitl_home_updated', {
        'lat': lat,
        'lon': lon,
        'alt': alt,
        'exe_path': runtime_state.sitl_exe_path or DEFAULT_SITL_EXE_PATH,
    })
    experiment_logger.log_event('set_sitl_home', {
        'lat': lat,
        'lon': lon,
        'alt': alt,
        'exe_path': runtime_state.sitl_exe_path or DEFAULT_SITL_EXE_PATH,
    })


@socketio.on('start_sitl')
def on_start_sitl(data):
    global sitl_process

    data = data or {}

    exe_path = data.get('exe_path') or runtime_state.sitl_exe_path or DEFAULT_SITL_EXE_PATH
    lat = float(data.get('lat', runtime_state.sitl_home_lat))
    lon = float(data.get('lon', runtime_state.sitl_home_lon))
    alt = float(data.get('alt', runtime_state.sitl_home_alt))
    runtime_state.sitl_exe_path = exe_path
    runtime_state.sitl_home_lat = lat
    runtime_state.sitl_home_lon = lon
    runtime_state.sitl_home_alt = alt

    if sitl_process and sitl_process.poll() is None:
        emit('status', {'msg': 'SITL is already running', 'type': 'warning'})
        socketio.emit('sitl_started', {
            'running': True,
            'pid': sitl_process.pid,
            'connection_string': runtime_state.connection_string or 'tcp:127.0.0.1:5760',
            'exe_path': exe_path,
            'lat': lat,
            'lon': lon,
            'alt': alt,
        })
        return

    try:
        sitl_process = _launch_sitl_process(exe_path, lat, lon, alt)
        runtime_state.connection_string = 'tcp:127.0.0.1:5760'
        emit('status', {
            'msg': f'SITL started at ({lat:.5f}, {lon:.5f}) @ {alt:.0f}m',
            'type': 'success',
        })
        socketio.emit('sitl_started', {
            'running': True,
            'pid': sitl_process.pid,
            'connection_string': runtime_state.connection_string,
            'exe_path': exe_path,
            'lat': lat,
            'lon': lon,
            'alt': alt,
        })
        experiment_logger.log_event('start_sitl', {
            'pid': sitl_process.pid,
            'exe_path': exe_path,
            'lat': lat,
            'lon': lon,
            'alt': alt,
        })
    except Exception as exc:
        sitl_process = None
        emit('status', {'msg': f'SITL start failed: {exc}', 'type': 'error'})
        experiment_logger.log_event('start_sitl_failed', {'error': str(exc)})


@socketio.on('stop_sitl')
def on_stop_sitl(_):
    global sitl_process

    if not sitl_process or sitl_process.poll() is not None:
        emit('status', {'msg': 'SITL is not running', 'type': 'warning'})
        socketio.emit('sitl_stopped', {'running': False})
        sitl_process = None
        return

    try:
        sitl_process.terminate()
        try:
            sitl_process.wait(timeout=10)
        except Exception:
            sitl_process.kill()
        emit('status', {'msg': 'SITL stopped', 'type': 'warning'})
        socketio.emit('sitl_stopped', {'running': False})
        experiment_logger.log_event('stop_sitl', {'pid': sitl_process.pid})
    finally:
        sitl_process = None


@socketio.on('arm')
def on_arm(_):
    if DRONEKIT_AVAILABLE and vehicle:
        vehicle.armed = True
    else:
        telemetry_state.armed = True

    emit('status', {'msg': 'Armed', 'type': 'success'})
    socketio.emit('armed_state', {'armed': True})
    experiment_logger.log_event('arm', {})


@socketio.on('disarm')
def on_disarm(_):
    if DRONEKIT_AVAILABLE and vehicle:
        vehicle.armed = False
    else:
        telemetry_state.armed = False
        telemetry_state.mode = 'STABILIZE'
        telemetry_state.alt = 0.0
        runtime_state.sim_target['alt'] = 0.0
        runtime_state.novice_auto_pending = False
        telemetry_state.wp_action_state = 'idle'

    emit('status', {'msg': 'Disarmed', 'type': 'warning'})
    socketio.emit('armed_state', {'armed': False})
    experiment_logger.log_event('disarm', {})


@socketio.on('takeoff')
def on_takeoff(data):
    alt = float(data.get('altitude', 20))

    if DRONEKIT_AVAILABLE and vehicle:
        vehicle.mode = VehicleMode('GUIDED')
        vehicle.simple_takeoff(alt)
    else:
        if not telemetry_state.armed:
            emit('status', {'msg': 'Arm first!', 'type': 'error'})
            return
        runtime_state.sim_target['alt'] = alt
        telemetry_state.mode = 'GUIDED'

    emit('status', {'msg': f'Taking off to {alt}m', 'type': 'success'})
    experiment_logger.log_event('takeoff', {'altitude': alt})


@socketio.on('land')
def on_land(_):
    if DRONEKIT_AVAILABLE and vehicle:
        vehicle.mode = VehicleMode('LAND')
    else:
        telemetry_state.mode = 'LAND'
        runtime_state.sim_target['alt'] = 0.0

    emit('status', {'msg': 'Landing initiated', 'type': 'warning'})
    experiment_logger.log_event('land', {})


@socketio.on('rtl')
def on_rtl(_):
    if DRONEKIT_AVAILABLE and vehicle:
        vehicle.mode = VehicleMode('RTL')
    else:
        telemetry_state.mode = 'RTL'
        runtime_state.sim_target.update({'lat': 28.6139, 'lon': 77.2090, 'alt': 0.0})

    emit('status', {'msg': 'Return to Launch', 'type': 'warning'})
    experiment_logger.log_event('rtl', {})


@socketio.on('goto')
def on_goto(data):
    lat = float(data.get('lat', telemetry_state.lat))
    lon = float(data.get('lon', telemetry_state.lon))
    alt = float(data.get('alt', 20))
    speed = float(data.get('speed', 5))

    if DRONEKIT_AVAILABLE and vehicle:
        vehicle.airspeed = speed
        vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt))
    else:
        runtime_state.sim_target.update({'lat': lat, 'lon': lon, 'alt': alt})
        telemetry_state.mode = 'GUIDED'

    emit('status', {'msg': f'GoTo ({lat:.5f}, {lon:.5f}) @ {alt}m', 'type': 'success'})
    experiment_logger.log_event('goto', {'lat': lat, 'lon': lon, 'alt': alt, 'speed': speed})


@socketio.on('set_mode')
def on_set_mode(data):
    mode = data.get('mode', 'STABILIZE')
    if DRONEKIT_AVAILABLE and vehicle:
        vehicle.mode = VehicleMode(mode)
    else:
        mode = str(mode).upper()
        telemetry_state.mode = mode
        runtime_state.novice_auto_pending = False

        if mode == 'AUTO':
            _set_sim_target_from_current_wp()
            telemetry_state.wp_action_state = 'moving'
        elif mode == 'LOITER':
            runtime_state.sim_target.update({
                'lat': telemetry_state.lat,
                'lon': telemetry_state.lon,
                'alt': telemetry_state.alt,
            })
            telemetry_state.wp_action_state = 'loitering'
            telemetry_state.wp_hold_start_sim_time = time.time()
        elif mode == 'RTL':
            runtime_state.sim_target.update({
                'lat': runtime_state.sitl_home_lat,
                'lon': runtime_state.sitl_home_lon,
                'alt': 0.0,
            })
            telemetry_state.wp_action_state = 'returning_home'
        elif mode == 'LAND':
            runtime_state.sim_target.update({
                'lat': telemetry_state.lat,
                'lon': telemetry_state.lon,
                'alt': 0.0,
            })
            telemetry_state.wp_action_state = 'landing'
        else:
            runtime_state.sim_target.update({
                'lat': telemetry_state.lat,
                'lon': telemetry_state.lon,
                'alt': telemetry_state.alt,
            })
            telemetry_state.wp_action_state = 'idle'

    emit('status', {'msg': f'Mode: {mode}', 'type': 'info'})
    experiment_logger.log_event('set_mode', {'mode': mode})


@socketio.on('upload_mission')
def on_upload_mission(data):
    waypoints = data.get('waypoints', [])
    mission_context = data.get('mission_context', {})
    if not DRONEKIT_AVAILABLE:
        mission_context.setdefault('remote_id_enabled', True)
        mission_context.setdefault('operator_approved', True)
    result = mission_service.upload(waypoints, mission_context=mission_context)

    if not result['ok']:
        emit('status', {'msg': result['message'], 'type': 'error'})
        socketio.emit('safety_precheck', result['precheck'])
        experiment_logger.log_event('upload_mission_rejected', result)
        return

    if DRONEKIT_AVAILABLE and vehicle:
        cmds = vehicle.commands
        cmds.clear()
        for wp in waypoints:
            action = str(wp.get('action', 'waypoint') or 'waypoint').lower()
            if action == 'disarm':
                emit('status', {'msg': 'Disarm is not supported as a mission item on real hardware.', 'type': 'error'})
                experiment_logger.log_event('upload_mission_rejected', {'reason': 'unsupported_action_disarm'})
                return

            if action in ('takeoff', 'takeoff_after_hold'):
                cmd_id = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            elif action in ('land', 'land_and_hold'):
                cmd_id = mavutil.mavlink.MAV_CMD_NAV_LAND
            elif action == 'loiter':
                cmd_id = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME if float(wp.get('hold_s', 0.0) or 0.0) > 0 else mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
            elif action == 'rtl':
                cmd_id = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
            else:
                cmd_id = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

            hold_s = float(wp.get('hold_s', 0.0) or 0.0)
            speed = float(wp.get('speed', 0.0) or 0.0)
            cmd = Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                cmd_id,
                0,
                1,
                hold_s,
                speed,
                0,
                0,
                wp['lat'],
                wp['lon'],
                wp['alt'],
            )
            cmds.add(cmd)
        cmds.upload()

    emit('status', {'msg': result['message'], 'type': 'success'})
    socketio.emit('mission_updated', {'waypoints': waypoints})
    socketio.emit('safety_precheck', result['precheck'])
    experiment_logger.log_event('upload_mission', {'waypoint_count': len(waypoints)})


@socketio.on('set_operator_mode')
def on_set_operator_mode(data):
    global novice_mode
    operator_type = data.get('operator_type', 'novice')  # 'novice' or 'expert'
    novice_mode = (operator_type == 'novice')
    msg = 'Novice mode: Auto arm, takeoff, and land' if novice_mode else 'Expert mode: Manual control'
    emit('status', {'msg': msg, 'type': 'info'})
    experiment_logger.log_event('set_operator_mode', {'operator_type': operator_type})


@socketio.on('start_mission')
def on_start_mission(data):
    global novice_mode
    
    # Check if novice mode is specified in the request
    is_novice = data.get('novice_mode', novice_mode) if isinstance(data, dict) else novice_mode
    
    if DRONEKIT_AVAILABLE and vehicle:
        # Real drone
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        _set_mission_start_index(cmds)

        if is_novice:
            # Novice: arm + takeoff now, AUTO will engage automatically once altitude is reached.
            vehicle.mode = VehicleMode('GUIDED')
            vehicle.armed = True
            first_wp_alt = telemetry_state.waypoints[0]['alt'] if telemetry_state.waypoints else 20
            req_alt = float(data.get('takeoff_altitude', first_wp_alt)) if isinstance(data, dict) else float(first_wp_alt)
            takeoff_alt = max(5.0, min(120.0, req_alt))
            runtime_state.sim_target['alt'] = takeoff_alt

            current_alt = float((vehicle.location.global_relative_frame.alt or 0.0))
            if current_alt < (takeoff_alt - 1.0):
                vehicle.simple_takeoff(takeoff_alt)
                runtime_state.novice_auto_pending = True
                emit('status', {
                    'msg': f'Armed and taking off to {takeoff_alt:.0f}m, mission will start automatically',
                    'type': 'success',
                })
            else:
                runtime_state.novice_auto_pending = False
                vehicle.mode = VehicleMode('AUTO')
                emit('status', {
                    'msg': 'Already at takeoff altitude, switching to AUTO and starting mission',
                    'type': 'success',
                })
        else:
            runtime_state.novice_auto_pending = False
            vehicle.mode = VehicleMode('AUTO')
    else:
        # Simulation mode
        if is_novice:
            result = mission_service.start_novice_sequence()
            emit('status', {'msg': result['message'], 'type': 'success'})
        else:
            result = mission_service.start()
            emit('status', {'msg': result['message'], 'type': 'success'})

        # In novice simulation flow, takeoff target is managed by start_novice_sequence.
        if not is_novice:
            _set_sim_target_from_current_wp()

    if is_novice:
        emit('status', {'msg': 'Novice mission armed; AUTO will engage after takeoff altitude is reached.', 'type': 'success'})
    else:
        emit('status', {'msg': 'Mission started (AUTO mode)', 'type': 'success'})
    experiment_logger.log_event('start_mission', {'novice_mode': is_novice})


@socketio.on('clear_mission')
def on_clear_mission(_):
    result = mission_service.clear()
    emit('status', {'msg': result['message'], 'type': 'info'})
    socketio.emit('mission_updated', {'waypoints': []})
    experiment_logger.log_event('clear_mission', {})


@socketio.on('plan_mission_llm')
def on_plan_mission_llm(data):
    """
    Generate mission through the modular LLM planning pipeline.
    Frontend can optionally pass `mock_response` to test parser/validator deterministically,
    or provide provider credentials for real LLM API calls.
    """
    user_request = data.get('user_request', '').strip()
    mock_response = data.get('mock_response', '').strip()
    provider = data.get('provider', '').strip()
    api_key = data.get('api_key', '').strip()
    model = data.get('model', '').strip()

    if not user_request:
        emit('status', {'msg': 'Missing mission request text.', 'type': 'error'})
        return

    mission_context = f"Current telemetry: lat={telemetry_state.lat}, lon={telemetry_state.lon}, alt={telemetry_state.alt}m"
    augmented_request = f"{user_request}\n{mission_context}"

    def call_model(prompt_text):
        # Priority order: explicit mock > provider credentials > fallback to local mock
        if mock_response:
            return mock_response
        
        if provider and api_key:
            try:
                return _call_llm_provider(prompt_text, provider, api_key, model)
            except Exception as e:
                emit('status', {'msg': f'LLM provider error: {str(e)}', 'type': 'error'})
                # Fall back to mock on error
                return _mock_llm_call(augmented_request)
        
        # Fallback to local mock when no provider configured
        return _mock_llm_call(augmented_request)

    result = llm_pipeline.generate_with_retries(augmented_request, call_model)
    experiment_logger.log_event('llm_plan_attempt', {
        'request': user_request, 
        'ok': result['ok'],
        'provider': provider or 'mock',
    })

    if not result['ok']:
        emit('status', {'msg': 'LLM mission generation failed validation.', 'type': 'error'})
        socketio.emit('llm_plan_result', result)
        return

    emit('status', {'msg': 'LLM mission generated and validated.', 'type': 'success'})
    socketio.emit('llm_plan_result', result)


@socketio.on('record_experiment_metric')
def on_record_experiment_metric(data):
    experiment_logger.log_metric(data)
    emit('status', {'msg': 'Experiment metric recorded.', 'type': 'info'})


@app.route('/')
def index():
    response = app.make_response(render_template('index.html'))
    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response


@app.route('/api/status')
def api_status():
    return jsonify(
        {
            'server': 'online',
            'dronekit': DRONEKIT_AVAILABLE,
            'connected': telemetry_state.connected,
            'pipeline': {
                'llm_planner': True,
                'safety_validator': True,
                'experiment_logger': True,
            },
        }
    )


@app.route('/api/analysis')
def api_analysis():
    """Run analyze_metrics.summarize() on current metrics.csv and return JSON results."""
    metrics_path = 'logs/metrics.csv'
    
    if not os.path.exists(metrics_path):
        return jsonify({
            'ok': False,
            'error': 'No metrics data available yet.',
            'summary': [],
            'pairwise_effects': [],
            'anova_results': [],
        }), 404
    
    try:
        # Load metrics rows
        rows = []
        with open(metrics_path, 'r', encoding='utf-8') as fh:
            reader = csv.DictReader(fh)
            rows = list(reader)
        
        if not rows:
            return jsonify({
                'ok': True,
                'error': 'No metric rows found.',
                'summary': [],
                'pairwise_effects': [],
                'anova_results': [],
            })
        
        # Import analyze functions
        try:
            from scripts.analyze_metrics import summarize
            summary_data = summarize(rows)
        except ImportError:
            return jsonify({
                'ok': False,
                'error': 'Could not import analyze_metrics module.',
                'summary': [],
                'pairwise_effects': [],
                'anova_results': [],
            }), 500
        
        # Extract summary, pairwise, anova from the returned tuple
        if isinstance(summary_data, tuple) and len(summary_data) >= 3:
            summary_rows, pairwise_rows, anova_rows = summary_data[0], summary_data[1], summary_data[2]
        else:
            summary_rows, pairwise_rows, anova_rows = [], [], []
        
        return jsonify({
            'ok': True,
            'error': None,
            'summary': summary_rows,
            'pairwise_effects': pairwise_rows,
            'anova_results': anova_rows,
            'metrics_count': len(rows),
        })
    
    except Exception as e:
        return jsonify({
            'ok': False,
            'error': str(e),
            'summary': [],
            'pairwise_effects': [],
            'anova_results': [],
        }), 500


if __name__ == '__main__':
    print('=' * 55)
    print('  GROUND CONTROL STATION  |  Modular GCS v2.0')
    print('=' * 55)
    print(f"  DroneKit : {'Available' if DRONEKIT_AVAILABLE else 'Simulation Mode'}")
    print('  Server   : http://localhost:5000')
    print('=' * 55)
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
