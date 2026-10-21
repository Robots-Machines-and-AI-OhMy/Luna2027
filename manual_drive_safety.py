#!/usr/bin/env python3
"""ROS 2 / Phoenix 6 lunar rover control with Lift/Hopper safety.

IMPORTANT: Bench-test with mechanisms unloaded. Not a safety-rated controller.
Calibration is session-only: re-calibrate after EVERY power cycle or reset.
Physical hard stops and normally-closed limit switches must be wired independently
to suitable safety circuitry. Python input is supplementary, not the final failsafe.

UDP packet: struct.pack('<Bh', identifier, value)
Axes 0-7; buttons use identifier 128+button_number.
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
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
from phoenix6.controls import VelocityVoltage, StaticBrake
from phoenix6.orchestra import Orchestra
from phoenix6.signals import NeutralModeValue

BUS = "RaptorCAN"
PORT = 5000
WATCHDOG_S = 0.5
CONTROL_PERIOD_S = 0.02
DEADZONE = 3000
TRIGGER_THRESHOLD = 10000
SPEED_STEP = 1.0
MAX_DRIVE_RPS = 20.0
MAX_MECHANISM_RPS = 20.0
DEFAULT_DRIVE_RPS = 8.0
DEFAULT_MECHANISM_RPS = 5.0
CALIBRATION_RPS = 1.0  # slow and supervised; adjust only after testing
POSITION_MARGIN_ROT = 0.1  # conservative margin; verify mechanical geometry

# Leave stall detection OFF until thresholds are measured and validated.
STALL_CURRENT_A = None
STALL_MAX_VELOCITY_RPS = None
STALL_CONFIRM_S = 0.25
STALL_MIN_COMMAND_RPS = 0.5

# Automatic reverse is OFF until safe reverse direction, speed and distance
# have been verified for both mechanisms. Stop/lockout still work.
AUTO_REVERSE_ENABLED = False
REVERSE_RPS = 0.5
REVERSE_SECONDS = 0.20
REVERSE_MAX_ROT = 0.10
LOCKOUT_SECONDS = 3.0

ROOT = Path(__file__).resolve().parent
LIMITS_FILE = ROOT / "mechanism_limits.json"
LOG_DIR = ROOT / "electrical_logs"
MOTOR_IDS = {
    "front_right": 1, "back_right": 2, "back_left": 3,
    "front_left": 4, "Lift": 5, "Miner": 6, "Hopper": 7,
}
BUTTON_A = 0
BUTTON_L1 = 6
BUTTON_R1 = 7
AXIS_R2 = 4
AXIS_L2 = 5

# Verify the direction of each motor with the mechanism disconnected.
# +1 means positive Talon position corresponds to "up"/"forward".
DIRECTION = {"Lift": 1, "Hopper": 1}

FIELDS = [
    "timestamp", "motor", "can_id", "command_rps", "supply_current_a",
    "stator_current_a", "supply_voltage_v", "motor_voltage_v",
    "velocity_rps", "position_rot", "device_temp_c", "duty_cycle",
    "safety_state", "status",
]


def read_signal(motor, getter):
    try:
        signal = getattr(motor, getter)()
        status = getattr(signal, "status", None)
        if status is not None and hasattr(status, "is_ok") and not status.is_ok():
            return None
        val = getattr(signal, "value", None)
        if val is None:
            val = getattr(signal, "value_as_double", None)
        if callable(val):
            val = val()
        val = float(val)
        return val if math.isfinite(val) else None
    except Exception:
        return None


class ElectricalLogger:
    def __init__(self, node):
        self.node = node
        self.stop_event = threading.Event()
        self.thread = None

    def start(self):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        path = LOG_DIR / "motor_readings.csv"
        with path.open("a", newline="", buffering=1) as fp:
            writer = csv.DictWriter(fp, fieldnames=FIELDS)
            if fp.tell() == 0:
                writer.writeheader()
            while not self.stop_event.is_set():
                stamp = dt.datetime.now().astimezone().isoformat(timespec="milliseconds")
                for name, motor in self.node.motors.items():
                    values = {}
                    for key, getter in [
                        ("supply_current_a", "get_supply_current"),
                        ("stator_current_a", "get_stator_current"),
                        ("supply_voltage_v", "get_supply_voltage"),
                        ("motor_voltage_v", "get_motor_voltage"),
                        ("velocity_rps", "get_velocity"),
                        ("position_rot", "get_position"),
                        ("device_temp_c", "get_device_temp"),
                        ("duty_cycle", "get_duty_cycle"),
                    ]:
                        values[key] = read_signal(motor, getter)
                    writer.writerow({
                        "timestamp": stamp, "motor": name, "can_id": MOTOR_IDS[name],
                        "command_rps": self.node.commanded.get(name, 0.0),
                        **values,
                        "safety_state": self.node.safety[name]["state"] if name in self.node.safety else "",
                        "status": "OK" if all(v is not None for v in values.values()) else "READ_ERROR",
                    })
                self.stop_event.wait(0.5)

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)


class ManualControlNode(Node):
    def __init__(self, calibrate=False):
        super().__init__("manual_control")
        self.calibrate = calibrate
        self.axes = [0] * 8
        self.buttons = [0] * 16
        self.drive_speed = DEFAULT_DRIVE_RPS
        self.mechanism_speed = DEFAULT_MECHANISM_RPS
        self.dpad_x_last = 0
        self.dpad_y_last = 0
        self.last_packet = 0.0
        self.connected = False
        self.commanded = {name: 0.0 for name in MOTOR_IDS}
        self.motors = {}
        bus = CANBus(BUS)
        for name, device_id in MOTOR_IDS.items():
            self.motors[name] = hardware.TalonFX(device_id, bus)
            time.sleep(0.12)  # Phoenix startup pacing used in earlier rover code
        # Preserve existing Talon configuration/gains. Only request brake mode.
        for name in ("Lift", "Hopper"):
            cfg = TalonFXConfiguration()
            cfg.motor_output.neutral_mode = NeutralModeValue.BRAKE
            # Do not overwrite known working PID gains with a blank config.
            # Set brake via Phoenix Tuner X before running if necessary.
        self.controls = {name: VelocityVoltage(0.0) for name in MOTOR_IDS}
        self.brake = StaticBrake()
        self.orchestra = Orchestra()
        try:
            self.orchestra.add_instrument(self.motors["Miner"])
            music = ROOT / "rickroll_loud.chrp"
            if music.exists():
                self.orchestra.load_music(str(music))
                self.orchestra.play()
        except Exception as exc:
            self.get_logger().warning(f"Music disabled: {exc}")

        self.saved_limits = self.load_limits()
        self.safety = {
            name: {
                "state": "READY", "stall_since": None, "lock_until": 0.0,
                "reverse_until": 0.0, "reverse_start": None,
                "reverse_command": 0.0, "cal_low": None, "cal_high": None,
                "cal_origin": None, "calibrated": False, "limits": None,
                "last_position": None,
            } for name in ("Lift", "Hopper")
        }
        self.switches = {
            name: {"low": False, "high": False} for name in ("Lift", "Hopper")
        }
        # IMPORTANT: no GPIO switch integration yet. Switches default to
        # NOT ACTIVE, so they provide NO protection until wired and tested.
        # Do not enable movement on the assumption these are working.
        self.create_timer(0.05, lambda: unmanaged.feed_enable(0.100))
        self.create_timer(CONTROL_PERIOD_S, self.control_tick)
        self.logger_thread = ElectricalLogger(self)
        self.logger_thread.start()
        self.get_logger().warning(
            "SAFETY: position limits are DISARMED until calibrated THIS SESSION. "
            "Stall thresholds and auto-reverse are disabled. GPIO switches NOT wired."
        )
        if calibrate:
            self.get_logger().warning(
                "CALIBRATION MODE: only Lift/Hopper can move; use L2/R2 and L1/R1. "
                "Never drive into hard stops. Press Ctrl+C to save sampled extrema."
            )

    @staticmethod
    def load_limits():
        try:
            return json.loads(LIMITS_FILE.read_text())
        except (OSError, ValueError):
            return {}

    def save_limits(self):
        data = {}
        for name, s in self.safety.items():
            if s["cal_low"] is not None and s["cal_high"] is not None:
                data[name] = {"low": s["cal_low"], "high": s["cal_high"],
                              "reference": "encoder_rotations_since_session_zero",
                              "note": "DO NOT trust after power cycle without homing"}
        if data:
            tmp = LIMITS_FILE.with_suffix(".tmp")
            tmp.write_text(json.dumps(data, indent=2))
            os.replace(tmp, LIMITS_FILE)
            self.get_logger().info(f"Saved calibration samples: {LIMITS_FILE}")

    def receive_packet(self, identifier, value):
        if 0 <= identifier < 8:
            self.axes[identifier] = int(value)
        elif 128 <= identifier < 144:
            self.buttons[identifier - 128] = int(value != 0)
        self.last_packet = time.monotonic()
        self.connected = True

    def clear_inputs(self):
        self.axes = [0] * 8
        self.buttons = [0] * 16

    def issue(self, name, speed, brake=False):
        self.commanded[name] = float(speed)
        self.controls[name].velocity = float(speed)
        if brake and speed == 0:
            self.motors[name].set_control(self.brake)
        else:
            self.motors[name].set_control(self.controls[name])

    def safety_trip(self, name, reason, attempted_speed, now):
        s = self.safety[name]
        if s["state"] != "READY":
            return
        self.get_logger().error(f"{name}: {reason}; stop and lockout")
        self.issue(name, 0.0)
        s["stall_since"] = None
        if AUTO_REVERSE_ENABLED and attempted_speed != 0:
            s["reverse_command"] = -math.copysign(REVERSE_RPS, attempted_speed)
            s["reverse_until"] = now + REVERSE_SECONDS
            s["reverse_start"] = read_signal(self.motors[name], "get_position")
            # Never reverse toward a pressed switch.
            target = "high" if s["reverse_command"] * DIRECTION[name] > 0 else "low"
            if self.switches[name][target]:
                s["state"] = "LOCKED"
                s["lock_until"] = now + LOCKOUT_SECONDS
            else:
                s["state"] = "REVERSING"
        else:
            s["state"] = "LOCKED"
            s["lock_until"] = now + LOCKOUT_SECONDS

    def guarded_mechanism(self, name, requested, now):
        s = self.safety[name]
        position = read_signal(self.motors[name], "get_position")
        speed = read_signal(self.motors[name], "get_velocity")
        current = read_signal(self.motors[name], "get_stator_current")
        if position is None or speed is None or current is None:
            self.issue(name, 0.0)
            s["state"] = "SENSOR_FAULT"
            return
        s["last_position"] = position
        if s["state"] == "SENSOR_FAULT":
            # Requires restart after diagnosing sensor/CAN issue.
            self.issue(name, 0.0)
            return
        if s["state"] == "REVERSING":
            if now >= s["reverse_until"] or (
                s["reverse_start"] is not None
                and abs(position - s["reverse_start"]) >= REVERSE_MAX_ROT
            ):
                self.issue(name, 0.0)
                s["state"] = "LOCKED"
                s["lock_until"] = now + LOCKOUT_SECONDS
                return
            reverse = s["reverse_command"]
            side = "high" if reverse * DIRECTION[name] > 0 else "low"
            if self.switches[name][side]:
                self.issue(name, 0.0)
                s["state"] = "LOCKED"
                s["lock_until"] = now + LOCKOUT_SECONDS
                return
            self.issue(name, reverse)
            return
        if s["state"] == "LOCKED":
            self.issue(name, 0.0)
            if now >= s["lock_until"]:
                s["state"] = "WAIT_RELEASE"
            return
        if s["state"] == "WAIT_RELEASE":
            self.issue(name, 0.0)
            if requested == 0:
                s["state"] = "READY"
            return

        if self.calibrate:
            # Origin is the encoder reading at calibration start, not a
            # hardware zero. Sample both extrema continuously.
            if s["cal_origin"] is None:
                s["cal_origin"] = position
            relative = position - s["cal_origin"]
            s["cal_low"] = min(relative, s["cal_low"] if s["cal_low"] is not None else relative)
            s["cal_high"] = max(relative, s["cal_high"] if s["cal_high"] is not None else relative)
            requested = max(-CALIBRATION_RPS, min(CALIBRATION_RPS, requested))
        else:
            # CRITICAL: refuse mechanism motion without a validated home
            # reference and limits established in this same session.
            if not s["calibrated"] or s["limits"] is None:
                self.issue(name, 0.0)
                return
            low, high = s["limits"]
            if requested * DIRECTION[name] < 0 and position <= low + POSITION_MARGIN_ROT:
                self.issue(name, 0.0)
                return
            if requested * DIRECTION[name] > 0 and position >= high - POSITION_MARGIN_ROT:
                self.issue(name, 0.0)
                return

        side = "high" if requested * DIRECTION[name] > 0 else "low"
        if requested != 0 and self.switches[name][side]:
            self.safety_trip(name, f"{side} limit switch", requested, now)
            return

        stall_enabled = STALL_CURRENT_A is not None and STALL_MAX_VELOCITY_RPS is not None
        if stall_enabled and abs(requested) >= STALL_MIN_COMMAND_RPS:
            if current >= STALL_CURRENT_A and abs(speed) <= STALL_MAX_VELOCITY_RPS:
                if s["stall_since"] is None:
                    s["stall_since"] = now
                elif now - s["stall_since"] >= STALL_CONFIRM_S:
                    self.safety_trip(name, "stall suspected", requested, now)
                    return
            else:
                s["stall_since"] = None
        else:
            s["stall_since"] = None
        self.issue(name, requested)

    def control_tick(self):
        now = time.monotonic()
        if not self.connected or now - self.last_packet > WATCHDOG_S:
            self.clear_inputs()
            for name in MOTOR_IDS:
                self.issue(name, 0.0, brake=(name == "Miner"))
            return
        # Edge-trigger D-pad speed adjustment.
        dx = -1 if self.axes[6] < -DEADZONE else 1 if self.axes[6] > DEADZONE else 0
        dy = -1 if self.axes[7] < -DEADZONE else 1 if self.axes[7] > DEADZONE else 0
        if dx and dx != self.dpad_x_last:
            self.mechanism_speed = min(MAX_MECHANISM_RPS, max(1, self.mechanism_speed + dx*SPEED_STEP))
        if dy and dy != self.dpad_y_last:
            self.drive_speed = min(MAX_DRIVE_RPS, max(1, self.drive_speed - dy*SPEED_STEP))
        self.dpad_x_last, self.dpad_y_last = dx, dy

        if not self.calibrate:
            x = self.axes[0] / 32767.0 if abs(self.axes[0]) > DEADZONE else 0.0
            y = -self.axes[1] / 32767.0 if abs(self.axes[1]) > DEADZONE else 0.0
            left = max(-1, min(1, y + x)) * self.drive_speed
            right = -max(-1, min(1, y - x)) * self.drive_speed
            for name in ("front_left", "back_left"):
                self.issue(name, left)
            for name in ("front_right", "back_right"):
                self.issue(name, right)
            self.issue("Miner", self.mechanism_speed if self.buttons[BUTTON_A] else 0.0,
                       brake=not self.buttons[BUTTON_A])
        else:
            for name in ("front_left", "back_left", "front_right", "back_right", "Miner"):
                self.issue(name, 0.0, brake=name == "Miner")

        l2 = self.axes[AXIS_L2] > TRIGGER_THRESHOLD
        r2 = self.axes[AXIS_R2] > TRIGGER_THRESHOLD
        lift = self.mechanism_speed * DIRECTION["Lift"] * ((1 if l2 else 0) - (1 if r2 else 0))
        hopper = self.mechanism_speed * DIRECTION["Hopper"] * (
            (1 if self.buttons[BUTTON_L1] else 0) - (1 if self.buttons[BUTTON_R1] else 0)
        )
        self.guarded_mechanism("Lift", lift, now)
        self.guarded_mechanism("Hopper", hopper, now)

    def stop_all(self):
        for name in MOTOR_IDS:
            try:
                self.issue(name, 0.0, brake=name == "Miner")
            except Exception:
                pass
        self.logger_thread.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibrate", action="store_true",
                        help="Supervised, slow mechanism-only sampling; save on Ctrl+C")
    args = parser.parse_args()
    rclpy.init()
    node = None
    sock = None
    try:
        node = ManualControlNode(calibrate=args.calibrate)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", PORT))
        sock.setblocking(False)
        node.get_logger().info(f"Listening on UDP {PORT} via {BUS}")
        while rclpy.ok():
            ready, _, _ = select.select([sock], [], [], 0.01)
            if ready:
                data, _ = sock.recvfrom(2048)
                if len(data) == 3:
                    identifier, value = struct.unpack("<Bh", data)
                    node.receive_packet(identifier, value)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.stop_all()
            if args.calibrate:
                node.save_limits()
            node.destroy_node()
        if sock:
            sock.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
