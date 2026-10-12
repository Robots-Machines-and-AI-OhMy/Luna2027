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
  AXIS 6  D-pad horizontal  Unassigned
  AXIS 7  D-pad vertical    Unassigned
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
DRIVE: Original arcade mixing and NEGATED RIGHT SIDE preserved.
MUSIC: rickroll_loud.chrp plays on startup, stops on active control.
SAFETY: 0.5-second packet watchdog. Sender must send regular packets
        even when joystick remains steady. Physical E-stop required.
        Lift travel limits are NOT implemented; test unloaded and secured.
        Miner closed-loop gains are NOT configured here; it must
        already have suitable gains on its Talon FX controller.
"""

import os
import select
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from phoenix6 import hardware, unmanaged
from phoenix6.canbus import CANBus
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityVoltage
from phoenix6.orchestra import Orchestra
from phoenix6.signals import InvertedValue, NeutralModeValue

# ----- Drive and mechanism speeds (rotations/second) -----
MAX_VELOCITY = 20.0       # Original working drive speed; unchanged
LIFT_VELOCITY = 10.0      # Test carefully: no software travel limits
MINER_VELOCITY = 10.0
HOPPER_VELOCITY = 10.0

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

        # Keep original per-side drive request objects and sign conventions.
        self.left_control = VelocityVoltage(0.0)
        self.right_control = VelocityVoltage(0.0)
        # Lift/Hopper: FOC off (Phoenix Pro license status unconfirmed).
        self.lift_control = VelocityVoltage(0.0).with_enable_foc(False)
        self.miner_control = VelocityVoltage(0.0)
        self.hopper_control = VelocityVoltage(0.0).with_enable_foc(False)

        # Phoenix must be fed while running outside the FRC robot framework.
        self.enable_timer = self.create_timer(0.05, self.feed_enable)
        self.last_log_time = 0.0
        self.get_logger().info('Ready: UDP controller on port 5000')

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
        self.axes[identifier] = value
        if identifier in (LEFT_X_AXIS, LEFT_Y_AXIS):
            self.update_motors()
        elif identifier in (R2_AXIS, L2_AXIS):
            self.update_lift()
        # Right stick and D-pad are tracked but intentionally unassigned.

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
        left_velocity = left * MAX_VELOCITY
        right_velocity = right * MAX_VELOCITY

        self.left_control.velocity = left_velocity
        self.right_control.velocity = right_velocity * -1  # DO NOT REMOVE
        self.motors['front_left'].set_control(self.left_control)
        self.motors['back_left'].set_control(self.left_control)
        self.motors['front_right'].set_control(self.right_control)
        self.motors['back_right'].set_control(self.right_control)

    def update_lift(self):
        """Hold L2 to raise, R2 to lower; neither/both stops CAN ID 5."""
        l2_pressed = self.axes[L2_AXIS] > TRIGGER_THRESHOLD
        r2_pressed = self.axes[R2_AXIS] > TRIGGER_THRESHOLD
        if l2_pressed and not r2_pressed:
            velocity = LIFT_VELOCITY
        elif r2_pressed and not l2_pressed:
            velocity = -LIFT_VELOCITY
        else:
            velocity = 0.0
        if velocity != 0.0:
            self.stop_music()
        elif self.orchestra.is_playing():
            return  # Idle heartbeat must not interrupt music (like drive)
        self.lift_control.velocity = velocity
        self.motors['Lift'].set_control(self.lift_control)

    def update_miner(self):
        """Hold A to run Miner (CAN ID 6); release to stop."""
        velocity = MINER_VELOCITY if self.buttons[BUTTON_A] else 0.0
        if velocity != 0.0:
            self.stop_music()
        elif self.orchestra.is_playing():
            return  # Idle heartbeat must not interrupt music (like drive)
        self.miner_control.velocity = velocity
        self.motors['Miner'].set_control(self.miner_control)

    def update_hopper(self):
        """Hold L1 forward, R1 reverse; neither/both stops CAN ID 7."""
        l1_pressed = self.buttons[BUTTON_L1]
        r1_pressed = self.buttons[BUTTON_R1]
        if l1_pressed and not r1_pressed:
            velocity = HOPPER_VELOCITY
        elif r1_pressed and not l1_pressed:
            velocity = -HOPPER_VELOCITY
        else:
            velocity = 0.0
        if velocity != 0.0:
            self.stop_music()
        elif self.orchestra.is_playing():
            return  # Idle heartbeat must not interrupt music (like drive)
        self.hopper_control.velocity = velocity
        self.motors['Hopper'].set_control(self.hopper_control)

    def zero_axes(self):
        """Watchdog: clear ALL inputs and stop all motors on packet loss."""
        self.axes = {axis: 0 for axis in AXIS_NAMES}
        self.axes[R2_AXIS] = -32767
        self.axes[L2_AXIS] = -32767
        self.buttons = {button: 0 for button in BUTTON_NAMES}
        self.stop_motor_commands()

    def stop_motor_commands(self):
        """Send zero velocity to all seven Talon FX controllers."""
        self.stop_music()
        self.left_control.velocity = 0.0
        self.right_control.velocity = 0.0
        self.lift_control.velocity = 0.0
        self.miner_control.velocity = 0.0
        self.hopper_control.velocity = 0.0
        for name in ('front_left', 'back_left'):
            self.motors[name].set_control(self.left_control)
        for name in ('front_right', 'back_right'):
            self.motors[name].set_control(self.right_control)
        self.motors['Lift'].set_control(self.lift_control)
        self.motors['Miner'].set_control(self.miner_control)
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
        if sock is not None:
            sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
