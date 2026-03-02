# Fix for the error: AttributeError: module 'collections' has no attribute 'MutableMapping'
from collections import abc, deque
import collections
collections.MutableMapping = abc.MutableMapping

import time
import math
import json
import threading
import queue
import shlex

from dronekit import connect, VehicleMode, LocationGlobalRelative
import tcp_relay

from flask import Flask, request, jsonify, Response, stream_with_context

# ── Flask app ─────────────────────────────────────────────────────────────────
flask_app = Flask(__name__)

# ── Adjustable runtime configuration (keep at top) ────────────────────────────
PWM_MIN = 1000
PWM_MAX = 2000
VEHICLE_CONNECTION_STRING = 'udpin:127.0.0.1:14540'
VEHICLE_BAUD = 57600
VEHICLE_RATE_HZ = 60
FLASK_HOST = '0.0.0.0'
FLASK_PORT = 5001
TCP_RELAY_HOST = '0.0.0.0'
TCP_RELAY_PORT = 1234
MAVLINK_MESSAGE_BUFFER_SIZE = 250
MAVLINK_LINK_TIMEOUT_SEC = 3.0
TERMINAL_DEFAULT_MODE = 'QHOVER'
TERMINAL_DEFAULT_TAKEOFF_ALT_M = 10.0
TERMINAL_DEFAULT_GOTO_ALT_M = 10.0
COMMAND_LONG_TARGET_SYSTEM_DEFAULT = 1
COMMAND_LONG_TARGET_COMPONENT_DEFAULT = 1

# ── Shared motor state ────────────────────────────────────────────────────────
motor_state = {
    "vtol": [0.0, 0.0, 0.0, 0.0],
    "forward": 0.0,
    "is_vtol": 0.0,
    "is_fw": 0.0,
}

# ── Shared live telemetry dict ────────────────────────────────────────────────
live_telemetry = {
    "roll": 0, "pitch": 0, "yaw": 0,
    "lat": 0, "lon": 0, "alt": 0,
    "battery": 0, "battery_remaining": 100,
    "ground_speed": 0, "airspeed": 0,
    "heading": 0, "throttle": 0,
    "armed": False, "mode": "UNKNOWN",
    "is_armable": False, "ekf_ok": False,
    "timestamp": 0, "server_latency_ms": 0,
    "motors": motor_state,
}
telemetry_lock = threading.Lock()

# ── MAVLink terminal message buffer ───────────────────────────────────────────
mavlink_messages = deque(maxlen=MAVLINK_MESSAGE_BUFFER_SIZE)
mavlink_messages_lock = threading.Lock()
last_mavlink_message_time = 0.0
heartbeat_last_mode = None
heartbeat_last_armed = None

# ── SSE subscriber management ─────────────────────────────────────────────────
sse_subscribers = []
sse_lock = threading.Lock()


def pwm_to_normalized(pwm):
    """Convert PWM (1000-2000) to 0.0-1.0. Returns 0.0 if disarmed/below threshold."""
    if pwm is None or pwm < 1050:
        return 0.0
    return max(0.0, min(1.0, (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)))


def _coerce_float(value, field_name):
    try:
        return float(value)
    except Exception:
        raise ValueError(f"Invalid {field_name}")


def _coerce_int(value, field_name):
    try:
        return int(float(value))
    except Exception:
        raise ValueError(f"Invalid {field_name}")


def _vehicle_mode_name():
    try:
        return vehicle.mode.name
    except Exception:
        return "UNKNOWN"


def append_mavlink_message(line, msg_type='INFO', payload=None):
    global last_mavlink_message_time

    entry = {
        'timestamp': time.time(),
        'type': str(msg_type),
        'line': str(line),
        'payload': payload if isinstance(payload, dict) else {},
    }

    with mavlink_messages_lock:
        mavlink_messages.append(entry)

    last_mavlink_message_time = entry['timestamp']


def broadcast_to_sse():
    """Push current telemetry to all connected SSE clients."""
    with telemetry_lock:
        data = live_telemetry.copy()

    payload = f"data: {json.dumps(data)}\n\n"

    with sse_lock:
        dead = []
        for subscriber_queue in sse_subscribers:
            try:
                subscriber_queue.put_nowait(payload)
            except queue.Full:
                dead.append(subscriber_queue)
        for subscriber_queue in dead:
            sse_subscribers.remove(subscriber_queue)


def execute_terminal_text(command_text):
    text = str(command_text or '').strip()
    if not text:
        raise ValueError('Command text is empty')

    tokens = shlex.split(text)
    if not tokens:
        raise ValueError('Command text is empty')

    command = tokens[0].lower()
    append_mavlink_message(f'TX> {text}', msg_type='TERMINAL_TX')

    if command in ('help', '?'):
        help_text = (
            'Supported: arm, disarm, mode <MODE>, takeoff [ALT], '
            'goto <LAT> <LON> [ALT], rtl, land, loiter, '
            'param get <NAME>, param set <NAME> <VALUE>, '
            'cmdlong <CMD_ID> [P1..P7]'
        )
        append_mavlink_message(help_text, msg_type='TERMINAL')
        return {'status': help_text, 'command': text}

    if command == 'arm':
        vehicle.armed = True
        status = 'armed'
    elif command == 'disarm':
        vehicle.armed = False
        status = 'disarmed'
    elif command == 'mode':
        mode = tokens[1].upper() if len(tokens) > 1 else TERMINAL_DEFAULT_MODE
        vehicle.mode = VehicleMode(mode)
        status = f'mode set to {mode}'
    elif command == 'takeoff':
        altitude = _coerce_float(tokens[1], 'altitude') if len(tokens) > 1 else TERMINAL_DEFAULT_TAKEOFF_ALT_M
        vehicle.simple_takeoff(altitude)
        status = f'taking off to {altitude}m'
    elif command in ('goto', 'guided_goto'):
        if len(tokens) < 3:
            raise ValueError('Usage: goto <lat> <lon> [alt]')
        lat = _coerce_float(tokens[1], 'latitude')
        lon = _coerce_float(tokens[2], 'longitude')
        alt = _coerce_float(tokens[3], 'altitude') if len(tokens) > 3 else TERMINAL_DEFAULT_GOTO_ALT_M
        vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt))
        status = f'going to {lat},{lon} at {alt}m'
    elif command == 'rtl':
        vehicle.mode = VehicleMode('RTL')
        status = 'mode set to RTL'
    elif command == 'land':
        vehicle.mode = VehicleMode('LAND')
        status = 'mode set to LAND'
    elif command == 'loiter':
        vehicle.mode = VehicleMode('LOITER')
        status = 'mode set to LOITER'
    elif command == 'param':
        if len(tokens) < 3:
            raise ValueError('Usage: param get <name> | param set <name> <value>')

        subcommand = tokens[1].lower()
        param_name = tokens[2]

        if subcommand == 'get':
            value = vehicle.parameters[param_name]
            status = f'{param_name}={value}'
            append_mavlink_message(status, msg_type='PARAM')
            return {'status': status, 'name': param_name, 'value': value, 'command': text}

        if subcommand == 'set':
            if len(tokens) < 4:
                raise ValueError('Usage: param set <name> <value>')
            value = _coerce_float(tokens[3], 'parameter value')
            vehicle.parameters[param_name] = value
            status = f'{param_name} set to {value}'
        else:
            raise ValueError('Usage: param get <name> | param set <name> <value>')
    elif command == 'cmdlong':
        if len(tokens) < 2:
            raise ValueError('Usage: cmdlong <cmd_id> [p1..p7]')

        command_id = _coerce_int(tokens[1], 'command id')
        params = []
        for token in tokens[2:9]:
            params.append(_coerce_float(token, 'command parameter'))
        while len(params) < 7:
            params.append(0.0)

        target_system = int(getattr(vehicle, 'target_system', COMMAND_LONG_TARGET_SYSTEM_DEFAULT) or COMMAND_LONG_TARGET_SYSTEM_DEFAULT)
        target_component = int(getattr(vehicle, 'target_component', COMMAND_LONG_TARGET_COMPONENT_DEFAULT) or COMMAND_LONG_TARGET_COMPONENT_DEFAULT)

        msg = vehicle.message_factory.command_long_encode(
            target_system,
            target_component,
            command_id,
            0,
            params[0], params[1], params[2], params[3], params[4], params[5], params[6]
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        status = f'cmdlong sent command={command_id} params={params}'
    else:
        raise ValueError('Unknown terminal command. Type "help" for supported commands.')

    append_mavlink_message(f'OK> {status}', msg_type='TERMINAL_RX')
    return {'status': status, 'command': text}


def execute_command_payload(data):
    command = str(data.get('command', '')).strip().lower()
    if not command:
        raise ValueError('missing command')

    if command == 'arm':
        vehicle.armed = True
        status = 'armed'
        append_mavlink_message('OK> armed', msg_type='COMMAND')
        return {'status': status}

    if command == 'disarm':
        vehicle.armed = False
        status = 'disarmed'
        append_mavlink_message('OK> disarmed', msg_type='COMMAND')
        return {'status': status}

    if command == 'mode':
        mode = str(data.get('mode', TERMINAL_DEFAULT_MODE)).upper()
        vehicle.mode = VehicleMode(mode)
        status = f'mode set to {mode}'
        append_mavlink_message(f'OK> {status}', msg_type='COMMAND')
        return {'status': status}

    if command == 'takeoff':
        altitude = _coerce_float(data.get('altitude', TERMINAL_DEFAULT_TAKEOFF_ALT_M), 'altitude')
        vehicle.simple_takeoff(altitude)
        status = f'taking off to {altitude}m'
        append_mavlink_message(f'OK> {status}', msg_type='COMMAND')
        return {'status': status}

    if command == 'guided_goto':
        lat = _coerce_float(data.get('lat'), 'latitude')
        lon = _coerce_float(data.get('lon'), 'longitude')
        altitude = _coerce_float(data.get('altitude', TERMINAL_DEFAULT_GOTO_ALT_M), 'altitude')
        vehicle.simple_goto(LocationGlobalRelative(lat, lon, altitude))
        status = f'going to {lat},{lon} at {altitude}m'
        append_mavlink_message(f'OK> {status}', msg_type='COMMAND')
        return {'status': status}

    if command == 'terminal':
        return execute_terminal_text(data.get('text', ''))

    raise ValueError('unknown command')


# ── Connect to vehicle ────────────────────────────────────────────────────────
vehicle = connect(VEHICLE_CONNECTION_STRING, wait_ready=True, baud=VEHICLE_BAUD, rate=VEHICLE_RATE_HZ)
print("Vehicle Connected.")
relay = tcp_relay.TCP_Relay(host=TCP_RELAY_HOST, port=TCP_RELAY_PORT)

# ── MAVLink listeners ─────────────────────────────────────────────────────────

@vehicle.on_message('ATTITUDE')
def attitude_listener(self, name, msg):
    with telemetry_lock:
        live_telemetry["roll"] = round(math.degrees(msg.roll), 2)
        live_telemetry["pitch"] = round(math.degrees(msg.pitch), 2)
        live_telemetry["yaw"] = round(math.degrees(msg.yaw), 2)
        live_telemetry["timestamp"] = time.time()
    broadcast_to_sse()


@vehicle.on_message('GLOBAL_POSITION_INT')
def position_listener(self, name, msg):
    with telemetry_lock:
        live_telemetry["lat"] = msg.lat / 1e7
        live_telemetry["lon"] = msg.lon / 1e7
        live_telemetry["alt"] = round(msg.relative_alt / 1000.0, 2)
        live_telemetry["heading"] = round(msg.hdg / 100.0, 1)
        live_telemetry["ground_speed"] = round(math.sqrt(msg.vx**2 + msg.vy**2) / 100.0, 2)
        live_telemetry["timestamp"] = time.time()
    broadcast_to_sse()


@vehicle.on_message('VFR_HUD')
def vfr_listener(self, name, msg):
    with telemetry_lock:
        live_telemetry["airspeed"] = round(msg.airspeed, 2)
        live_telemetry["ground_speed"] = round(msg.groundspeed, 2)
        live_telemetry["throttle"] = round(msg.throttle, 1)
        live_telemetry["alt"] = round(msg.alt, 2)
        live_telemetry["heading"] = msg.heading
        live_telemetry["timestamp"] = time.time()
    broadcast_to_sse()


@vehicle.on_message('SYS_STATUS')
def battery_listener(self, name, msg):
    with telemetry_lock:
        live_telemetry["battery"] = round(msg.voltage_battery / 1000.0, 2)
        live_telemetry["battery_remaining"] = msg.battery_remaining
        live_telemetry["timestamp"] = time.time()
    broadcast_to_sse()


@vehicle.on_message('HEARTBEAT')
def heartbeat_listener(self, name, msg):
    global heartbeat_last_mode, heartbeat_last_armed

    with telemetry_lock:
        live_telemetry["armed"] = vehicle.armed
        live_telemetry["mode"] = _vehicle_mode_name()
        live_telemetry["is_armable"] = vehicle.is_armable
        live_telemetry["ekf_ok"] = vehicle.ekf_ok
        live_telemetry["motors"] = motor_state.copy()
        live_telemetry["timestamp"] = time.time()

    current_mode = _vehicle_mode_name()
    current_armed = bool(vehicle.armed)
    if heartbeat_last_mode != current_mode:
        append_mavlink_message(f'Got HEARTBEAT: MODE {current_mode}', msg_type='HEARTBEAT', payload={'mode': current_mode})
        heartbeat_last_mode = current_mode
    if heartbeat_last_armed != current_armed:
        append_mavlink_message(
            f'Got HEARTBEAT: ARM {"ARMED" if current_armed else "DISARMED"}',
            msg_type='HEARTBEAT',
            payload={'armed': current_armed}
        )
        heartbeat_last_armed = current_armed

    broadcast_to_sse()


@vehicle.on_message('STATUSTEXT')
def statustext_listener(self, name, msg):
    text = str(getattr(msg, 'text', '')).strip()
    severity = int(getattr(msg, 'severity', -1))
    if text:
        append_mavlink_message(
            f'Got STATUSTEXT [SEV{severity}]: {text}',
            msg_type='STATUSTEXT',
            payload={'severity': severity, 'text': text}
        )


@vehicle.on_message('COMMAND_ACK')
def command_ack_listener(self, name, msg):
    command = int(getattr(msg, 'command', -1))
    result = int(getattr(msg, 'result', -1))
    append_mavlink_message(
        f'Got COMMAND_ACK: command={command} result={result}',
        msg_type='COMMAND_ACK',
        payload={'command': command, 'result': result}
    )


@vehicle.on_message('SERVO_OUTPUT_RAW')
def servo_listener(self, name, msg):
    mode = _vehicle_mode_name()

    motor_state["vtol"][0] = pwm_to_normalized(msg.servo5_raw)
    motor_state["vtol"][1] = pwm_to_normalized(msg.servo6_raw)
    motor_state["vtol"][2] = pwm_to_normalized(msg.servo7_raw)
    motor_state["vtol"][3] = pwm_to_normalized(msg.servo8_raw)

    if mode in ['CRUISE', 'FBWA', 'FBWB', 'AUTO', 'RTL', 'AUTOTUNE']:
        motor_state["forward"] = pwm_to_normalized(msg.servo3_raw)
        motor_state["is_fw"] = 1.0
        motor_state["is_vtol"] = 0.0
    elif mode.startswith('Q'):
        motor_state["forward"] = 0.0
        motor_state["is_vtol"] = 1.0
        motor_state["is_fw"] = 0.0
    else:
        motor_state["forward"] = pwm_to_normalized(msg.servo3_raw)
        motor_state["is_vtol"] = 1.0
        motor_state["is_fw"] = 1.0


# ── Flask routes ──────────────────────────────────────────────────────────────

@flask_app.route('/status', methods=['GET'])
def status_endpoint():
    """Polling endpoint for backward compatibility."""
    with telemetry_lock:
        data = live_telemetry.copy()
    return json.dumps(data)


@flask_app.route('/mavlink_messages', methods=['GET'])
def mavlink_messages_endpoint():
    with mavlink_messages_lock:
        messages = list(mavlink_messages)
    return jsonify({
        'online': (time.time() - last_mavlink_message_time) < MAVLINK_LINK_TIMEOUT_SEC,
        'endpoint': VEHICLE_CONNECTION_STRING,
        'count': len(messages),
        'messages': messages,
        'server_time': time.time(),
    })


@flask_app.route('/telemetry/stream')
def telemetry_sse():
    """SSE endpoint — connects once, pushes on every MAVLink packet."""
    subscriber_queue = queue.Queue(maxsize=10)
    with sse_lock:
        sse_subscribers.append(subscriber_queue)

    def generate():
        try:
            while True:
                try:
                    data = subscriber_queue.get(timeout=2.0)
                    yield data
                except queue.Empty:
                    yield ": keepalive\n\n"
        finally:
            with sse_lock:
                try:
                    sse_subscribers.remove(subscriber_queue)
                except ValueError:
                    pass

    return Response(
        stream_with_context(generate()),
        mimetype='text/event-stream',
        headers={
            'Cache-Control': 'no-cache',
            'X-Accel-Buffering': 'no',
            'Connection': 'keep-alive'
        }
    )


@flask_app.route('/command', methods=['POST'])
def send_mavlink_command():
    """Send a MAVLink command via POST request."""
    data = request.json or {}

    try:
        result = execute_command_payload(data)
        return jsonify(result)
    except ValueError as exc:
        append_mavlink_message(f'ERR> {exc}', msg_type='TERMINAL_ERROR')
        return jsonify({'error': str(exc)}), 400
    except Exception as exc:
        append_mavlink_message(f'ERR> {exc}', msg_type='TERMINAL_ERROR')
        return jsonify({'error': 'command execution failed', 'detail': str(exc)}), 500


@flask_app.route('/params', methods=['GET'])
def get_params():
    """Get all vehicle parameters."""
    params = {}
    for key, val in vehicle.parameters.items():
        params[key] = val
    return jsonify(params)


@flask_app.route('/param', methods=['POST'])
def set_param():
    """Set a specific parameter."""
    data = request.json or {}
    name = data.get('name')
    value = data.get('value')
    if name and value is not None:
        vehicle.parameters[name] = value
        append_mavlink_message(f'OK> {name} set to {value}', msg_type='PARAM')
        return jsonify({"status": f"{name} set to {value}"})
    return jsonify({"error": "missing name or value"}), 400


# ── UE5 helper ────────────────────────────────────────────────────────────────

def vehicle_to_unreal(vehicle, z_invert=True, scale=100):
    data = {}
    data["lat"] = vehicle.location.global_frame.lat
    data["lon"] = vehicle.location.global_frame.lon
    data["alt"] = vehicle.location.global_frame.alt
    data["n"] = vehicle.location.local_frame.north * scale
    data["e"] = vehicle.location.local_frame.east * scale
    data["d"] = vehicle.location.local_frame.down * scale
    if z_invert:
        data["d"] *= -1
    data["roll"] = vehicle.attitude.roll
    data["pitch"] = vehicle.attitude.pitch
    data["yaw"] = vehicle.attitude.yaw

    for key, value in data.items():
        if isinstance(value, float):
            if key in ["lat", "lon", "alt"]:
                data[key] = round(value, 8)
            elif key in ["n", "e", "d"]:
                data[key] = round(value, 3)
            elif key in ["roll", "pitch", "yaw"]:
                data[key] = round(math.degrees(value), 3)
    return data


# ── Start Flask in background thread ─────────────────────────────────────────

def start_flask():
    flask_app.run(host=FLASK_HOST, port=FLASK_PORT, debug=False, threaded=True)


flask_thread = threading.Thread(target=start_flask, daemon=True)
flask_thread.start()
print(f"MAVLink API running on http://localhost:{FLASK_PORT}")
print(f"SSE telemetry stream at http://localhost:{FLASK_PORT}/telemetry/stream")
append_mavlink_message('MAVLink bridge ready', msg_type='INFO')


# ── Main loop — UE5 relay ─────────────────────────────────────────────────────

while True:
    data = vehicle_to_unreal(vehicle)

    fields = [0.0] * relay.num_fields

    fields[0] = data["n"]
    fields[1] = data["e"]
    fields[2] = data["d"]
    fields[3] = data["roll"]
    fields[4] = data["pitch"]
    fields[5] = data["yaw"]

    fields[6] = motor_state["vtol"][0]
    fields[7] = motor_state["vtol"][1]
    fields[8] = motor_state["vtol"][2]
    fields[9] = motor_state["vtol"][3]

    fields[10] = motor_state["forward"]

    if motor_state["is_vtol"] and motor_state["is_fw"]:
        fields[11] = 3.0
    elif motor_state["is_vtol"]:
        fields[11] = 1.0
    elif motor_state["is_fw"]:
        fields[11] = 2.0
    else:
        fields[11] = 0.0

    relay.message = tcp_relay.create_fields_string(fields)
    time.sleep(1 / 60)
