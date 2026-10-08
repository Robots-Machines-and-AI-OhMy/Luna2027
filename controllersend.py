#!/usr/bin/env python3
"""Read the 8BitDo joystick and send axis/button state to the Jetson over UDP.

UDP packet: struct.pack('<Bh', identifier, value)
  AXES:    identifiers 0-7, signed values approximately -32767..32767
  BUTTONS: identifiers 128 + button number, value 0 (released) or 1 (pressed)
"""

import argparse
import select
import socket
import struct
import sys
import time

#Linux joystick event format
JS_EVENT_FORMAT = "<IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02

#Original controller mappings
LEFT_X_AXIS = 0
LEFT_Y_AXIS = 1
R2_AXIS = 4
L2_AXIS = 5

#Monitor all axes 0-7 (receiver uses 0/1 for drive, 4/5 for Lift)
WATCHED_AXES = set(range(8))

#Analog triggers rest near -32767, not 0
TRIGGER_REST = -32767

#Buttons are sent as 128 + button number
BUTTON_PACKET_OFFSET = 128

#UDP packet format
PACKET_FORMAT = "<Bh"
HEARTBEAT_INTERVAL = 0.05


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        "--device",
        default="/dev/input/js0",
        help="Joystick device node"
    )

    parser.add_argument(
        "--remote-ip",
        default="10.42.0.2",
        help="Edge device (Jetson) IP"
    )

    parser.add_argument(
        "--remote-port",
        type=int,
        default=5000
    )

    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    remote = (args.remote_ip, args.remote_port)
    print(
        f"Reading {args.device}, sending axis/button updates "
        f"to {args.remote_ip}:{args.remote_port}"
    )

    # Initial joystick positions (triggers at rest, not 0)
    state = {axis: 0 for axis in WATCHED_AXES}
    state[R2_AXIS] = TRIGGER_REST
    state[L2_AXIS] = TRIGGER_REST

    # Button states, filled in as the driver reports them
    buttons = {}

    with open(args.device, "rb") as joystick:
        fd = joystick.fileno()
        next_heartbeat = time.monotonic()

        while True:
            timeout = max(
                0.0,
                next_heartbeat - time.monotonic()
            )

            ready, _, _ = select.select([fd], [], [], timeout)

            if fd in ready:
                event = joystick.read(JS_EVENT_SIZE)

                if len(event) < JS_EVENT_SIZE:
                    break

                timestamp, value, ev_type, number = struct.unpack(
                    JS_EVENT_FORMAT,
                    event
                )

                if (ev_type & JS_EVENT_AXIS) and number in WATCHED_AXES:
                    state[number] = value

                    packet = struct.pack(
                        PACKET_FORMAT,
                        number,
                        value
                    )

                    sock.sendto(packet, remote)

                elif (ev_type & JS_EVENT_BUTTON) and number < BUTTON_PACKET_OFFSET:
                    buttons[number] = 1 if value else 0

                    packet = struct.pack(
                        PACKET_FORMAT,
                        BUTTON_PACKET_OFFSET + number,
                        buttons[number]
                    )

                    sock.sendto(packet, remote)

            now = time.monotonic()

            if now >= next_heartbeat:
                for axis, value in state.items():
                    packet = struct.pack(
                        PACKET_FORMAT,
                        axis,
                        value
                    )

                    sock.sendto(packet, remote)

                # Resend buttons too, so a lost release packet can't
                # leave the Miner running
                for button, value in buttons.items():
                    packet = struct.pack(
                        PACKET_FORMAT,
                        BUTTON_PACKET_OFFSET + button,
                        value
                    )

                    sock.sendto(packet, remote)

                next_heartbeat = now + HEARTBEAT_INTERVAL

    sock.close()


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        pass

    except FileNotFoundError as exc:
        print(
            f"Controller device not found: {exc}",
            file=sys.stderr
        )
        sys.exit(1)
