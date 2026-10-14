#!/usr/bin/env python3
"""Lunar mining rover — ROS 2 / Phoenix 6 / UDP 8BitDo manual control.

CONTROLLER COMPUTER: 10.42.0.1, running controller_send.py
JETSON: receives UDP on port 5000, CAN bus RaptorCAN

UDP packet: struct.pack('<Bh', identifier, value)
  AXES: identifiers 0–7, signed values approximately -32767..32767
  BUTTONS: identifiers 128 + button number, value 0 (released) or 1 (pressed)
IMPORTANT: The sender MUST implement the button-identifier convention.
An older axis-only sender still drives, but A will not operate the Miner.

8BITDO BUTTON LAYOUT (Linux input numbers, verified on user's controller)
  AXIS 0  Left stick X      Turn left/right
  AXIS 1  Left stick Y      Drive forward/reverse
  AXIS 2  Right stick X     Unassigned
  AXIS 3  Right stick Y     Unassigned
  AXIS 4  R2 analog         Hold to LOWER Lift (CAN 5)
  AXIS 5  L2 analog         Hold to RAISE Lift (CAN 5)
  AXIS 6  D-pad horizontal  Right/left: mechanism speed +/-1 RPS
  AXIS 7  D-pad vertical    Up/down: drive speed +/-1 RPS
  BUTTON 0  A               Hold to run Miner (CAN 6)
  BUTTON 1  B               Unassigned
  BUTTON 2  R4              Unassigned
  BUTTON 3  X               Unassigned
  BUTTON 4  Y               Unassigned
  BUTTON 5  L4              Unassigned
  BUTTON 6  L1              Hold to run Hopper FORWARD (CAN 7)
  BUTTON 7  R1              Hold to run Hopper REVERSE (CAN 7)
  BUTTON 8  L2 digital      Unused; analog axis 5 operates Lift
  BUTTON 9  R2 digital      Unused; analog axis 4 operates Lift
  BUTTON 13 L3              Unassigned
  BUTTON 14 R3              Unassigned

MOTORS: 1 front_right, 2 back_right, 3 back_left, 4 front_left,
        5 Lift, 6 Miner, 7 Hopper.
LIFT/HOPPER: Kraken X44s, configured at startup (X44 gains, current
        limits, brake mode). VelocityVoltage with FOC disabled.
MECHANISM IDLE: Lift and Hopper (lead screws) receive a zero-velocity
        closed-loop command when released. Miner receives a Phoenix 6
        StaticBrake request (motor leads shorted) when A is released.
DRIVE: Original arcade mixing and NEGATED RIGHT SIDE preserved.
MUSIC: rickroll_loud.chrp plays on startup, stops on active control.
SAFETY: 0.5-second packet watchdog. Sender must send regular packets
        even when joystick remains steady. Physical E-stop required.
        Lift travel limits are NOT implemented; test unloaded and secured.
        Miner closed-loop gains are NOT configured here; it must
        already have suitable gains on its Talon FX controller.
"""

import csv
import datetime as dt
import os
import queue
import select
import socket
import struct
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from phoenix6 import hardware, unmanaged
from phoenix6.canbus import CANBus
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import StaticBrake, VelocityVoltage
from phoenix6.orchestra import Orchestra
from phoenix6.signals import InvertedValue, NeutralModeValue

# ----- Drive and mechanism speeds (rotations/second) -----
MAX_VELOCITY = 20.0       # Original working drive speed; unchanged
LIFT_VELOCITY = 10.0      # Test carefully: no software travel limits
MINER_VELOCITY = 10.0
HOPPER_VELOCITY = 10.0
SPEED_STEP = 1.0
DRIVE_SPEED_LIMIT = 30.0  # Adjustable ceiling; validate before using above 20
MECHANISM_SPEED_LIMIT = 20.0  # Adjustable ceiling; validate before using above 10

# ----- Lift/Hopper Kraken X44 config (~7,500 RPM free ≈ 126 rps at 12 V) -----
X44_K_S = 0.10            # V to overcome static friction
X44_K_V = 0.095           # V per rps (12 V / ~126 rps)
X44_K_P = 0.11            # V per rps of error
X44_STATOR_LIMIT = 40.0   # A
X44_SUPPLY_LIMIT = 30.0   # A
LIFT_INVERTED = False     # Flip if L2 lowers instead of raises
HOPPER_INVERTED = False   # Flip if L1 runs Hopper backwards
DEADZONE = 3000           # Ignore small left-stick movements
TRIGGER_THRESHOLD = -25000  # Trigger released near -32767

# ----- Complete joystick axis map -----
LEFT_X_AXIS = 0
LEFT_Y_AXIS = 1
RIGHT_X_AXIS = 2
RIGHT_Y_AXIS = 3
R2_AXIS = 4
L2_AXIS = 5
DPAD_X_AXIS = 6
DPAD_Y_AXIS = 7

# ----- Complete button map -----
BUTTON_A = 0
BUTTON_B = 1
BUTTON_R4 = 2
BUTTON_X = 3
BUTTON_Y = 4
BUTTON_L4 = 5
BUTTON_L1 = 6
BUTTON_R1 = 7
BUTTON_L2 = 8        # Digital event; not used for Lift
BUTTON_R2 = 9        # Digital event; not used for Lift
BUTTON_L3 = 13
BUTTON_R3 = 14

AXIS_NAMES = {
    LEFT_X_AXIS: 'Left X', LEFT_Y_AXIS: 'Left Y',
    RIGHT_X_AXIS: 'Right X', RIGHT_Y_AXIS: 'Right Y',
    R2_AXIS: 'R2', L2_AXIS: 'L2',
    DPAD_X_AXIS: 'D-pad X', DPAD_Y_AXIS: 'D-pad Y',
}
BUTTON_NAMES = {
    BUTTON_A: 'A', BUTTON_B: 'B', BUTTON_R4: 'R4',
    BUTTON_X: 'X', BUTTON_Y: 'Y', BUTTON_L4: 'L4',
    BUTTON_L1: 'L1', BUTTON_R1: 'R1',
    BUTTON_L2: 'L2 digital', BUTTON_R2: 'R2 digital',
    BUTTON_L3: 'L3', BUTTON_R3: 'R3',
}

# The original UDP protocol carries an identifier, not an event type.
# Keep axes 0..7 intact and encode buttons as 128 + their button number.
BUTTON_PACKET_OFFSET = 128
PACKET_FORMAT = '<Bh'  # unsigned byte identifier, signed 16-bit value
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
CONTROLLER_IP = '10.42.0.1'
CONTROLLER_PORT = 5000
STALE_TIMEOUT = 0.5
LOG_INTERVAL = 1.0
PHOENIX_STARTUP_DELAY = 2.0  # Seconds for Phoenix/CAN devices to come up

# Preserve Orchestra behavior and track mapping from uploaded code.
CHIRP_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'rickroll_loud.chrp'
)
INSTRUMENT_TRACKS = [
    ('front_left', 0), ('front_right', 0), ('Lift', 0),
    ('back_left', 1), ('back_right', 1),
    ('Miner', 2), ('Hopper', 2),
]


# ----- Electrical logger (inlined from electrical_logger.py) -----
# Non-blocking CSV diagnostics for Phoenix 6 Talon FX motors on Linux.
FIELDS = ["timestamp", "elapsed_s", "motor", "can_id", "command", "supply_current_a", "stator_current_a", "supply_voltage_v", "motor_voltage_v", "velocity_rps", "position_rot", "device_temp_c", "duty_cycle", "status"]
FAULT_FIELDS = ["timestamp", "motor", "can_id", "kind", "details"]
README = '''ROVER ELECTRICAL LOGS\n=====================\nmotor_readings.csv: one row per motor per sample (default 0.5 seconds).\nmotor_faults.csv: fault snapshots when they change, plus read/communication errors.\n\nCOLUMNS\ntimestamp: local timestamp with timezone.\nelapsed_s: seconds since logger start (monotonic).\nmotor: descriptive motor name.\ncan_id: Talon FX device ID, if accessible.\ncommand: latest command supplied by your control code; blank if not supplied.\nsupply_current_a: current drawn by motor controller from power supply.\nstator_current_a: current in motor windings; may exceed supply current.\nsupply_voltage_v: voltage seen at Talon FX supply input.\nmotor_voltage_v: voltage applied to motor (controller estimate).\nvelocity_rps: motor rotations per second, sign indicates direction.\nposition_rot: motor sensor position in rotations.\ndevice_temp_c: Talon FX device temperature (not necessarily motor winding temperature).\nduty_cycle: motor output fraction, generally -1 to +1.\nstatus: OK or failed signal names; failed reads should not be treated as zero.\n\nINTERPRETATION\nVoltage drop plus brownout fault: investigate wiring, connections, power distribution, battery.\nHigh stator current with near-zero velocity: possible mechanical stall or load.\nCommand present but no voltage/output: investigate controls, enable state, limits and faults.\nRead errors: investigate CANivore/CAN bus wiring and communication.\nCompare all readings at the same timestamp; one sample alone does not prove root cause.\nSticky faults persist until cleared; this logger never clears them.\n'''

class ElectricalLogger:
    def __init__(self, motors, directory="electrical_logs", interval=0.5, command_getter=None):
        """motors: dict of {name: TalonFX}. command_getter(name) optional."""
        self.motors = dict(motors)
        self.directory = Path(directory).expanduser().resolve()
        self.interval = max(0.1, float(interval))
        self.command_getter = command_getter
        self.stop_event = threading.Event()
        self.thread = None
        self.last_faults = {}
        self.start_time = time.monotonic()

    @staticmethod
    def _read(motor, method):
        try:
            signal = getattr(motor, method)()
            status = getattr(signal, "status", None)
            if status is not None and hasattr(status, "is_ok") and not status.is_ok():
                return "", f"{method}: {status}"
            value = getattr(signal, "value", None)
            if value is None:
                value = getattr(signal, "value_as_double", None)
            if callable(value):
                value = value()
            if value is None:
                return "", f"{method}: no value"
            return float(value), None
        except Exception as exc:
            return "", f"{method}: {type(exc).__name__}: {exc}"

    @staticmethod
    def _fault_snapshot(motor):
        out = {}
        for name in ("get_fault_field", "get_sticky_fault_field"):
            try:
                fn = getattr(motor, name, None)
                if fn is not None:
                    sig = fn()
                    status = getattr(sig, "status", None)
                    if status is not None and hasattr(status, "is_ok") and not status.is_ok():
                        out[name] = f"READ_ERROR {status}"
                    else:
                        out[name] = str(getattr(sig, "value", sig))
            except Exception as exc:
                out[name] = f"READ_ERROR {exc}"
        # Fallback: inspect standard Phoenix 6 fault getter methods if field getter unavailable.
        if not out:
            for name in dir(motor):
                if not (name.startswith("get_fault_") or name.startswith("get_sticky_fault_")):
                    continue
                try:
                    sig = getattr(motor, name)()
                    val = getattr(sig, "value", None)
                    if val is True or (isinstance(val, (int, float)) and val != 0):
                        out[name] = str(val)
                except Exception:
                    pass
        return out

    def start(self):
        if self.thread is not None:
            return
        self.directory.mkdir(parents=True, exist_ok=True)
        (self.directory / "README.txt").write_text(README, encoding="utf-8")
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, name="electrical-logger", daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3)
            self.thread = None

    def _run(self):
        readings = self.directory / "motor_readings.csv"
        faults = self.directory / "motor_faults.csv"
        with readings.open("a", newline="", buffering=1) as rf, faults.open("a", newline="", buffering=1) as ff:
            rw, fw = csv.DictWriter(rf, FIELDS), csv.DictWriter(ff, FAULT_FIELDS)
            if rf.tell() == 0: rw.writeheader()
            if ff.tell() == 0: fw.writeheader()
            methods = {"supply_current_a": "get_supply_current", "stator_current_a": "get_stator_current", "supply_voltage_v": "get_supply_voltage", "motor_voltage_v": "get_motor_voltage", "velocity_rps": "get_velocity", "position_rot": "get_position", "device_temp_c": "get_device_temp", "duty_cycle": "get_duty_cycle"}
            while not self.stop_event.is_set():
                started = time.monotonic()
                stamp = dt.datetime.now().astimezone().isoformat(timespec="milliseconds")
                for name, motor in self.motors.items():
                    can_id = getattr(motor, "device_id", "")
                    row = {"timestamp": stamp, "elapsed_s": round(started-self.start_time, 3), "motor": name, "can_id": can_id, "command": "", "status": "OK"}
                    errors = []
                    if self.command_getter:
                        try: row["command"] = self.command_getter(name)
                        except Exception as exc: errors.append(f"command: {exc}")
                    for column, method in methods.items():
                        row[column], err = self._read(motor, method)
                        if err: errors.append(err)
                    if errors:
                        row["status"] = "; ".join(errors)
                        fw.writerow({"timestamp": stamp, "motor": name, "can_id": can_id, "kind": "READ_ERROR", "details": row["status"]})
                    rw.writerow(row)
                    snapshot = self._fault_snapshot(motor)
                    if snapshot != self.last_faults.get(name):
                        fw.writerow({"timestamp": stamp, "motor": name, "can_id": can_id, "kind": "FAULT_CHANGE", "details": repr(snapshot)})
                        self.last_faults[name] = snapshot
                self.stop_event.wait(max(0, self.interval-(time.monotonic()-started)))


class ManualControlNode(Node):
    """Receive controller state and issue Phoenix 6 motor commands."""

    def __init__(self):
        super().__init__('manual_control')
        self.get_logger().info('Starting lunar rover manual control...')

        # Keep the SAME CANBus object type as the working uploaded code.
        self.canbus = CANBus('RaptorCAN')
        self.motors = {
            'front_right': hardware.TalonFX(1, self.canbus),
            'back_right': hardware.TalonFX(2, self.canbus),
            'back_left': hardware.TalonFX(3, self.canbus),
            'front_left': hardware.TalonFX(4, self.canbus),
            'Lift': hardware.TalonFX(5, self.canbus),
            'Miner': hardware.TalonFX(6, self.canbus),
            'Hopper': hardware.TalonFX(7, self.canbus),
        }

        # Give Phoenix time to finish setup before configuring or playing music.
        self.get_logger().info(
            f'Waiting {PHOENIX_STARTUP_DELAY}s for Phoenix setup...'
        )
        time.sleep(PHOENIX_STARTUP_DELAY)

        # EXACT drive gains from the uploaded working code.
        # Apply only to drive motors; mechanism tuning is separate.
        motor_config = TalonFXConfiguration()
        motor_config.slot0.k_p = 0.11
        motor_config.slot0.k_i = 0.0
        motor_config.slot0.k_d = 0.0
        motor_config.slot0.k_v = 0.12
        motor_config.slot0.k_s = 0.05
        motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        for name in ('front_right', 'back_right', 'back_left', 'front_left'):
            status = self.motors[name].configurator.apply(motor_config)
            if not status.is_ok():
                self.get_logger().error(f'Failed to configure {name}: {status}')

        # Lift/Hopper: Kraken X44 gains, current limits, brake mode.
        for name, inverted in (('Lift', LIFT_INVERTED),
                               ('Hopper', HOPPER_INVERTED)):
            x44_config = TalonFXConfiguration()
            x44_config.slot0.k_s = X44_K_S
            x44_config.slot0.k_v = X44_K_V
            x44_config.slot0.k_p = X44_K_P
            x44_config.slot0.k_i = 0.0
            x44_config.slot0.k_d = 0.0
            x44_config.current_limits.stator_current_limit = X44_STATOR_LIMIT
            x44_config.current_limits.stator_current_limit_enable = True
            x44_config.current_limits.supply_current_limit = X44_SUPPLY_LIMIT
            x44_config.current_limits.supply_current_limit_enable = True
            x44_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
            x44_config.motor_output.inverted = (
                InvertedValue.CLOCKWISE_POSITIVE if inverted
                else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
            status = self.motors[name].configurator.apply(x44_config)
            if not status.is_ok():
                self.get_logger().error(f'Failed to configure {name}: {status}')

        # Music uses all seven motors, as in the uploaded code.
        self.orchestra = Orchestra()
        for name, track in INSTRUMENT_TRACKS:
            self.orchestra.add_instrument_with_track(self.motors[name], track)
        status = self.orchestra.load_music(CHIRP_FILE)
        if status.is_ok():
            self.orchestra.play()
            self.get_logger().info(f'Playing {CHIRP_FILE}; move joystick to stop')
        else:
            self.get_logger().error(f'Failed to load {CHIRP_FILE}: {status}')

        # Store all axes/buttons, including currently unassigned ones.
        self.axes = {axis: 0 for axis in AXIS_NAMES}
        self.axes[R2_AXIS] = -32767
        self.axes[L2_AXIS] = -32767
        self.buttons = {button: 0 for button in BUTTON_NAMES}
        self.drive_speed = MAX_VELOCITY
        self.mechanism_speed = LIFT_VELOCITY

        # Keep original per-side drive request objects and sign conventions.
        self.left_control = VelocityVoltage(0.0)
        self.right_control = VelocityVoltage(0.0)
        # Lift/Hopper: FOC off (Phoenix Pro license status unconfirmed).
        self.lift_control = VelocityVoltage(0.0).with_enable_foc(False)
        self.miner_control = VelocityVoltage(0.0)
        self.hopper_control = VelocityVoltage(0.0).with_enable_foc(False)
        # Idle request for the Miner when A is released.
        self.brake_control = StaticBrake()

        # Phoenix must be fed while running outside the FRC robot framework.
        self.enable_timer = self.create_timer(0.05, self.feed_enable)
        self.last_log_time = 0.0
        # Electrical diagnostics for all seven motors; logs are written on Jetson.
        self.electrical_logger = ElectricalLogger(
            motors=self.motors,
            directory=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'electrical_logs'),
            interval=0.5,
            command_getter=self.motor_command_rps,
        )
        try:
            self.electrical_logger.start()
            self.get_logger().info('Electrical logs enabled in electrical_logs/')
        except OSError as exc:
            self.get_logger().error(f'Could not start electrical logging: {exc}')

        self.get_logger().info('Ready: UDP controller on port 5000')

    def motor_command_rps(self, name):
        """Requested motor velocity in rotations/sec, not measured speed."""
        if name in ('front_left', 'back_left'):
            return self.left_control.velocity
        if name in ('front_right', 'back_right'):
            return self.right_control.velocity
        return {
            'Lift': self.lift_control,
            'Miner': self.miner_control,
            'Hopper': self.hopper_control,
        }[name].velocity

    def feed_enable(self):
        """Keep Phoenix devices enabled while the ROS timer is serviced."""
        unmanaged.feed_enable(0.100)

    def normalize_axis(self, value):
        """Scale joystick -32767..32767 to -1..1, applying deadzone."""
        if abs(value) < DEADZONE:
            return 0.0
        value = value / 32767.0
        return max(-1.0, min(1.0, value))

    def stop_music(self):
        """Release Orchestra's control before issuing mechanism commands."""
        if self.orchestra.is_playing():
            self.orchestra.stop()

    def process_packet(self, data):
        """Decode one UDP packet, update the matching motor only."""
        if len(data) != PACKET_SIZE:
            return
        identifier, value = struct.unpack(PACKET_FORMAT, data)

        now = time.monotonic()
        if now - self.last_log_time >= LOG_INTERVAL:
            if identifier >= BUTTON_PACKET_OFFSET:
                name = BUTTON_NAMES.get(identifier - BUTTON_PACKET_OFFSET, 'unknown')
                self.get_logger().info(f'button {name}: {value}')
            else:
                name = AXIS_NAMES.get(identifier, 'unknown')
                self.get_logger().info(f'axis {identifier} ({name}): {value}')
            self.last_log_time = now

        # Button packets MUST be sent as 128 + button number.
        if identifier >= BUTTON_PACKET_OFFSET:
            button = identifier - BUTTON_PACKET_OFFSET
            if button not in self.buttons:
                return
            self.buttons[button] = 1 if value else 0
            if button == BUTTON_A:
                self.update_miner()
            elif button in (BUTTON_L1, BUTTON_R1):
                self.update_hopper()
            # All other buttons are intentionally unassigned.
            return

        # Axes retain the original 0..7 identifiers.
        if identifier not in self.axes:
            return
        previous = self.axes[identifier]
        self.axes[identifier] = value
        if identifier in (DPAD_X_AXIS, DPAD_Y_AXIS):
            self.adjust_speed(identifier, previous, value)
        if identifier in (LEFT_X_AXIS, LEFT_Y_AXIS):
            self.update_motors()
        elif identifier in (R2_AXIS, L2_AXIS):
            self.update_lift()
        # Right stick remains unassigned.

    def adjust_speed(self, axis, previous, value):
        """One speed change per D-pad press, not per repeated packet."""
        direction = 1 if value > 16000 else -1 if value < -16000 else 0
        old_direction = 1 if previous > 16000 else -1 if previous < -16000 else 0
        if direction == 0 or direction == old_direction:
            return
        if axis == DPAD_Y_AXIS:
            # Linux D-pad up is negative.
            delta = -direction * SPEED_STEP
            self.drive_speed = max(0.0, min(DRIVE_SPEED_LIMIT, self.drive_speed + delta))
            self.get_logger().info(f"Drive speed: {self.drive_speed:.1f} RPS")
            self.update_motors()
        elif axis == DPAD_X_AXIS:
            delta = direction * SPEED_STEP
            self.mechanism_speed = max(0.0, min(MECHANISM_SPEED_LIMIT, self.mechanism_speed + delta))
            self.get_logger().info(f"Lift/Miner/Hopper speed: {self.mechanism_speed:.1f} RPS")
            self.update_lift()
            self.update_miner()
            self.update_hopper()

    def update_motors(self):
        """Original working arcade drive; right motor sign remains inverted."""
        x = self.normalize_axis(self.axes[LEFT_X_AXIS])
        y = -self.normalize_axis(self.axes[LEFT_Y_AXIS])
        left = y + x
        right = y - x

        # Preserve original behavior: zero stick doesn't interrupt music.
        if self.orchestra.is_playing():
            if left == 0.0 and right == 0.0:
                return
            self.get_logger().info('Joystick input - stopping music')
            self.orchestra.stop()

        maximum = max(1.0, abs(left), abs(right))
        left /= maximum
        right /= maximum
        left_velocity = left * self.drive_speed
        right_velocity = right * self.drive_speed

        self.left_control.velocity = left_velocity
        self.right_control.velocity = right_velocity * -1  # DO NOT REMOVE
        self.motors['front_left'].set_control(self.left_control)
        self.motors['back_left'].set_control(self.left_control)
        self.motors['front_right'].set_control(self.right_control)
        self.motors['back_right'].set_control(self.right_control)

    def command_mechanism(self, name, control, velocity, static_brake=False):
        """Run a mechanism at velocity; when idle, hold zero velocity,
        or StaticBrake it if static_brake is True."""
        if velocity != 0.0:
            self.stop_music()
        elif self.orchestra.is_playing():
            return  # Idle heartbeat must not interrupt music (like drive)
        # Keep velocity current even when braking so the logged command is 0.
        control.velocity = velocity
        if velocity == 0.0 and static_brake:
            self.motors[name].set_control(self.brake_control)
            return
        self.motors[name].set_control(control)

    def update_lift(self):
        """Hold L2 to raise, R2 to lower; neither/both stops CAN ID 5."""
        l2_pressed = self.axes[L2_AXIS] > TRIGGER_THRESHOLD
        r2_pressed = self.axes[R2_AXIS] > TRIGGER_THRESHOLD
        if l2_pressed and not r2_pressed:
            velocity = self.mechanism_speed
        elif r2_pressed and not l2_pressed:
            velocity = -self.mechanism_speed
        else:
            velocity = 0.0
        self.command_mechanism('Lift', self.lift_control, velocity)

    def update_miner(self):
        """Hold A to run Miner (CAN ID 6); release to brake."""
        velocity = self.mechanism_speed if self.buttons[BUTTON_A] else 0.0
        self.command_mechanism('Miner', self.miner_control, velocity,
                               static_brake=True)

    def update_hopper(self):
        """Hold L1 forward, R1 reverse; neither/both stops CAN ID 7."""
        l1_pressed = self.buttons[BUTTON_L1]
        r1_pressed = self.buttons[BUTTON_R1]
        if l1_pressed and not r1_pressed:
            velocity = self.mechanism_speed
        elif r1_pressed and not l1_pressed:
            velocity = -self.mechanism_speed
        else:
            velocity = 0.0
        self.command_mechanism('Hopper', self.hopper_control, velocity)

    def zero_axes(self):
        """Watchdog: clear ALL inputs and stop all motors on packet loss."""
        self.axes = {axis: 0 for axis in AXIS_NAMES}
        self.axes[R2_AXIS] = -32767
        self.axes[L2_AXIS] = -32767
        self.buttons = {button: 0 for button in BUTTON_NAMES}
        self.stop_motor_commands()

    def stop_motor_commands(self):
        """Zero velocity to drive, Lift and Hopper; StaticBrake the Miner."""
        self.stop_music()
        self.left_control.velocity = 0.0
        self.right_control.velocity = 0.0
        self.lift_control.velocity = 0.0
        self.miner_control.velocity = 0.0  # Logged command; Miner is braked
        self.hopper_control.velocity = 0.0
        for name in ('front_left', 'back_left'):
            self.motors[name].set_control(self.left_control)
        for name in ('front_right', 'back_right'):
            self.motors[name].set_control(self.right_control)
        self.motors['Lift'].set_control(self.lift_control)
        self.motors['Miner'].set_control(self.brake_control)
        self.motors['Hopper'].set_control(self.hopper_control)

    def stop_all(self):
        """Stop motors and Orchestra when the process exits."""
        self.get_logger().info('STOPPING ALL MOTORS')
        self.stop_motor_commands()
        unmanaged.feed_enable(0.0)


def main(args=None):
    """ROS 2 event loop and nonblocking UDP receiver with packet watchdog."""
    rclpy.init(args=args)
    node = ManualControlNode()
    sock = None
    last_packet_time = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', CONTROLLER_PORT))
        sock.setblocking(False)
        node.get_logger().info(
            f'Listening on UDP 0.0.0.0:{CONTROLLER_PORT}; '
            f'only accepting {CONTROLLER_IP}'
        )
        while rclpy.ok():
            # Run Phoenix feed_enable timer even when no UDP packets arrive.
            rclpy.spin_once(node, timeout_sec=0.0)
            try:
                readable, _, exceptional = select.select([sock], [], [sock], 0.01)
            except (OSError, ValueError) as exc:
                node.get_logger().warn(f'select error: {exc}')
                continue
            if sock in readable:
                try:
                    while True:
                        data, address = sock.recvfrom(4096)
                        if address[0] != CONTROLLER_IP:
                            continue
                        # Ignore malformed packets without resetting watchdog.
                        if len(data) != PACKET_SIZE:
                            continue
                        node.process_packet(data)
                        last_packet_time = time.monotonic()
                except BlockingIOError:
                    pass
                except OSError as exc:
                    node.get_logger().warn(f'Socket error: {exc}')
            if sock in exceptional:
                node.get_logger().warn('Controller socket exception')
            if (last_packet_time is not None
                    and time.monotonic() - last_packet_time > STALE_TIMEOUT):
                node.get_logger().warn('Controller packets stale - stopping motors')
                node.zero_axes()
                last_packet_time = None
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        if hasattr(node, 'electrical_logger'):
            node.electrical_logger.stop()
        if sock is not None:
            sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
