from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

from chassis_controller import ChassisAck, ChassisController, ChassisFrame, ChassisStatus


def describe_frame(controller: ChassisController, frame: ChassisFrame) -> str:
    parsed = controller.parse_frame_payload(frame)
    prefix = f"seq={frame.seq:02X} cmd=0x{frame.cmd:02X} raw={controller.frame_to_hex(frame)}"
    if isinstance(parsed, ChassisStatus):
        return (
            f"{prefix} STATUS(mode={parsed.mode}, yaw={parsed.yaw_deg:.3f}, "
            f"vx={parsed.vx:.3f}, vy={parsed.vy:.3f}, wz={parsed.wz_deg_s:.3f}, "
            f"t={parsed.timestamp_ms}ms)"
        )
    if isinstance(parsed, ChassisAck):
        kind = "ACK" if frame.cmd == controller.responses["ack"] else "ERR"
        return f"{prefix} {kind}(original_cmd=0x{parsed.original_cmd:02X}, error=0x{parsed.error:02X})"
    return f"{prefix} payload={parsed.hex(' ').upper()}"


def send_and_print(controller: ChassisController, name: str, payload: bytes = b"", timeout_s: float = 0.4) -> list[ChassisFrame]:
    tx = controller.send_command(name, payload)
    print(f"\nTX {name}: {controller.frame_to_hex(tx)}")
    frames = controller.collect_frames(timeout_s)
    if not frames:
        print("RX: <timeout>")
        return []
    for frame in frames:
        print(f"RX {describe_frame(controller, frame)}")
    return frames


def has_status(controller: ChassisController, frames: list[ChassisFrame]) -> bool:
    return any(frame.cmd == controller.responses["status"] for frame in frames)


def main() -> int:
    parser = argparse.ArgumentParser(description="Test upper/lower controller serial protocol without running the chassis.")
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("communication.yml"))
    parser.add_argument("--port", help="Override serial port, for example COM5 or /dev/ttyUSB0.")
    parser.add_argument("--timeout", type=float, default=0.5, help="Receive window per test step in seconds.")
    parser.add_argument("--set-yaw", type=float, default=None, help="Optional yaw value to test SET_YAW.")
    parser.add_argument("--feedback-ms", type=int, default=0, help="Feedback interval to test, 0 disables periodic feedback.")
    parser.add_argument(
        "--unsafe-run-test",
        action="store_true",
        help="Also send a zero-speed RUN command. Wheels should be off the ground if this is changed later.",
    )
    args = parser.parse_args()

    controller = ChassisController(args.config)
    if args.port:
        controller.config["serial"]["port"] = args.port

    print(f"Opening {controller.config['serial']['port']} at {controller.config['serial']['baudrate']} baud")

    try:
        with controller:
            ok = True

            frames = send_and_print(controller, "hello", timeout_s=args.timeout)
            ok = ok and has_status(controller, frames)

            frames = send_and_print(controller, "get_status", timeout_s=args.timeout)
            ok = ok and has_status(controller, frames)

            payload = struct.pack("<H", max(0, min(65535, args.feedback_ms)))
            frames = send_and_print(controller, "set_feedback", payload, timeout_s=args.timeout)
            ok = ok and bool(frames)

            if args.set_yaw is not None:
                frames = send_and_print(controller, "set_yaw", struct.pack("<f", args.set_yaw), timeout_s=args.timeout)
                ok = ok and has_status(controller, frames)

            if args.unsafe_run_test:
                frames = send_and_print(controller, "run", struct.pack("<fff", 0.0, 0.0, 0.0), timeout_s=args.timeout)
                ok = ok and has_status(controller, frames)

            frames = send_and_print(controller, "brake", timeout_s=args.timeout)
            ok = ok and has_status(controller, frames)

            print("\nListening for extra frames for 0.5s...")
            time.sleep(0.1)
            extra = controller.collect_frames(0.5)
            if extra:
                for frame in extra:
                    print(f"RX {describe_frame(controller, frame)}")
            else:
                print("RX: <none>")

            if ok:
                print("\nPASS: serial protocol parsed valid responses.")
                return 0
            print("\nFAIL: at least one required STATUS response was not received.")
            return 1

    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
