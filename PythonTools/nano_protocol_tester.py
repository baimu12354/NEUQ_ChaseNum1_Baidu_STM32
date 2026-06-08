import argparse
import struct
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("请先安装 pyserial: pip install pyserial") from exc


HEAD = b"\xAA\x55"
TAIL = b"\x0D\x0A"
VERSION = 0x01

MODULE_SYSTEM = 0x01
MODULE_CHASSIS = 0x02
MODULE_ARM = 0x03
MODULE_PUMP = 0x04

CMD_SYS_HELLO = 0x01
CMD_SYS_RESET = 0x02
CMD_SYS_ESTOP = 0x03
CMD_SYS_GET_STATUS = 0x04

CMD_CHASSIS_RUN = 0x10
CMD_CHASSIS_STOP = 0x11
CMD_CHASSIS_BRAKE = 0x12
CMD_CHASSIS_ROTATE_TO = 0x13
CMD_CHASSIS_TRANSLATE_REL = 0x14
CMD_CHASSIS_GET_STATE = 0x15

CMD_ARM_TARGET_XYZ_DIR = 0x20
CMD_ARM_FOLD_HOME = 0x21
CMD_ARM_STOP_HOLD = 0x22
CMD_ARM_GET_POSE = 0x23

CMD_PUMP_SUCK = 0x30
CMD_PUMP_RELEASE = 0x31
CMD_PUMP_GET_STATE = 0x32

RESP_ACK = 0x80
RESP_NACK = 0x81
RESP_STATUS = 0x82
RESP_EVENT = 0x83
RESP_ARM_POSE = 0x84
RESP_CHASSIS_STATE = 0x85
RESP_PUMP_STATE = 0x86

MODULE_NAMES = {
    MODULE_SYSTEM: "SYSTEM",
    MODULE_CHASSIS: "CHASSIS",
    MODULE_ARM: "ARM",
    MODULE_PUMP: "PUMP",
}

RESP_NAMES = {
    RESP_ACK: "ACK",
    RESP_NACK: "NACK",
    RESP_STATUS: "STATUS",
    RESP_EVENT: "EVENT",
    RESP_ARM_POSE: "ARM_POSE",
    RESP_CHASSIS_STATE: "CHASSIS_STATE",
    RESP_PUMP_STATE: "PUMP_STATE",
}


class NanoProtocolTester:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.05):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.seq = 0
        self.rx_buf = bytearray()

    def close(self):
        self.ser.close()

    def next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    @staticmethod
    def checksum(seq: int, module: int, cmd: int, payload: bytes) -> int:
        return (VERSION + seq + module + cmd + len(payload) + sum(payload)) & 0xFF

    def build_frame(self, module: int, cmd: int, payload: bytes = b"", seq=None) -> bytes:
        if seq is None:
            seq = self.next_seq()
        body = bytes([VERSION, seq, module, cmd, len(payload)]) + payload
        check = self.checksum(seq, module, cmd, payload)
        return HEAD + body + bytes([check]) + TAIL

    def send(self, module: int, cmd: int, payload: bytes = b"") -> int:
        seq = self.next_seq()
        frame = self.build_frame(module, cmd, payload, seq)
        self.ser.write(frame)
        self.ser.flush()
        print(f"TX seq={seq:02X} module={MODULE_NAMES.get(module, module)} cmd=0x{cmd:02X} payload={payload.hex(' ')}")
        return seq

    def read_frames(self, duration: float = 0.5):
        end_time = time.time() + duration
        frames = []
        while time.time() < end_time:
            data = self.ser.read(128)
            if data:
                self.rx_buf.extend(data)
                frames.extend(self.extract_frames())
        return frames

    def extract_frames(self):
        frames = []
        while True:
            head_index = self.rx_buf.find(HEAD)
            if head_index < 0:
                self.rx_buf.clear()
                return frames
            if head_index > 0:
                del self.rx_buf[:head_index]
            if len(self.rx_buf) < 9:
                return frames

            length = self.rx_buf[6]
            frame_len = 2 + 5 + length + 1 + 2
            if len(self.rx_buf) < frame_len:
                return frames

            frame = bytes(self.rx_buf[:frame_len])
            del self.rx_buf[:frame_len]

            if frame[-2:] != TAIL:
                continue
            version, seq, module, cmd, payload_len = frame[2], frame[3], frame[4], frame[5], frame[6]
            payload = frame[7:7 + payload_len]
            check = frame[7 + payload_len]
            if version != VERSION:
                continue
            if check != self.checksum(seq, module, cmd, payload):
                print(f"RX bad checksum: {frame.hex(' ')}")
                continue
            frames.append((seq, module, cmd, payload))
        return frames

    @staticmethod
    def f32(*values: float) -> bytes:
        return struct.pack("<" + "f" * len(values), *values)

    @staticmethod
    def parse_frame(seq: int, module: int, cmd: int, payload: bytes) -> str:
        name = RESP_NAMES.get(cmd, f"0x{cmd:02X}")
        mod_name = MODULE_NAMES.get(module, f"0x{module:02X}")
        base = f"RX seq={seq:02X} module={mod_name} resp={name} len={len(payload)}"

        if cmd in (RESP_ACK, RESP_NACK) and len(payload) >= 5:
            ref_seq, ref_module, ref_cmd, result, state = payload[:5]
            return f"{base} ref_seq={ref_seq:02X} ref_module={MODULE_NAMES.get(ref_module, ref_module)} ref_cmd=0x{ref_cmd:02X} result=0x{result:02X} state={state}"

        if cmd == RESP_EVENT and len(payload) >= 5:
            ref_seq, event_module, event_id, state, result = payload[:5]
            return f"{base} ref_seq={ref_seq:02X} event_module={MODULE_NAMES.get(event_module, event_module)} event=0x{event_id:02X} state={state} result=0x{result:02X}"

        if cmd == RESP_STATUS and len(payload) >= 16:
            sys_s, chassis_s, arm_s, pump_s, movement, pump_on, sucked, arm_busy = payload[:8]
            tick = struct.unpack_from("<I", payload, 8)[0]
            yaw = struct.unpack_from("<f", payload, 12)[0]
            return f"{base} sys={sys_s} chassis={chassis_s} arm={arm_s} pump={pump_s} movement={movement} pump_on={pump_on} sucked={sucked} arm_busy={arm_busy} tick={tick} yaw={yaw:.2f}"

        if cmd == RESP_ARM_POSE and len(payload) >= 36:
            arm_state, pose_source, arm_busy, _ = payload[:4]
            values = struct.unpack_from("<ffffffff", payload, 4)
            x, y, z, direction, th0, th1, th2, th3 = values
            return f"{base} state={arm_state} source={pose_source} busy={arm_busy} xyz=({x:.2f},{y:.2f},{z:.2f}) dir={direction:.2f} th=({th0:.2f},{th1:.2f},{th2:.2f},{th3:.2f})"

        if cmd == RESP_CHASSIS_STATE and len(payload) >= 22:
            chassis_state, movement = payload[:2]
            yaw, vx, vy, wz, dist = struct.unpack_from("<fffff", payload, 2)
            return f"{base} state={chassis_state} movement={movement} yaw={yaw:.2f} vx={vx:.2f} vy={vy:.2f} wz={wz:.2f} dist_cm={dist:.2f}"

        if cmd == RESP_PUMP_STATE and len(payload) >= 10:
            pump_state, pump_on, sucked, _ = payload[:4]
            freq = struct.unpack_from("<f", payload, 4)[0]
            strength = payload[8]
            return f"{base} state={pump_state} on={pump_on} sucked={sucked} freq={freq:.2f} strength={strength}"

        return f"{base} payload={payload.hex(' ')}"

    def print_rx(self, duration: float = 0.5):
        for frame in self.read_frames(duration):
            print(self.parse_frame(*frame))


def main():
    parser = argparse.ArgumentParser(description="NANO UART2 unified protocol tester")
    parser.add_argument("--port", required=True, help="串口号，例如 COM8 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--listen", type=float, default=0.8, help="每次发送后监听秒数")
    parser.add_argument("command", choices=[
        "hello", "status", "reset", "estop",
        "chassis_state", "run", "stop", "brake", "rotate", "translate",
        "arm_pose", "arm_fold", "arm_stop", "arm_target",
        "pump_state", "pump_suck", "pump_release",
    ])
    parser.add_argument("--vx", type=float, default=0.0, help="running 横向速度，上位机量程 -100~100")
    parser.add_argument("--vy", type=float, default=0.0, help="running 纵向速度，上位机量程 -100~100")
    parser.add_argument("--wz", type=float, default=0.0, help="running/rotate 角速度，上位机量程 -100~100 或 1~100")
    parser.add_argument("--angle", type=float, default=0.0, help="定角旋转角度，单位 deg")
    parser.add_argument("--speed", type=float, default=20.0, help="平移/旋转速度，上位机量程 1~100")
    parser.add_argument("--dx", type=float, default=0.0, help="定向平移 X 距离，单位 cm，量程 -1000~1000")
    parser.add_argument("--dy", type=float, default=0.0, help="定向平移 Y 距离，单位 cm，量程 -1000~1000")
    parser.add_argument("--x", type=float, default=20.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=25.0)
    parser.add_argument("--dir", type=float, default=0.0)
    args = parser.parse_args()

    tester = NanoProtocolTester(args.port, args.baud)
    try:
        if args.command == "hello":
            tester.send(MODULE_SYSTEM, CMD_SYS_HELLO)
        elif args.command == "status":
            tester.send(MODULE_SYSTEM, CMD_SYS_GET_STATUS)
        elif args.command == "reset":
            tester.send(MODULE_SYSTEM, CMD_SYS_RESET)
        elif args.command == "estop":
            tester.send(MODULE_SYSTEM, CMD_SYS_ESTOP)
        elif args.command == "chassis_state":
            tester.send(MODULE_CHASSIS, CMD_CHASSIS_GET_STATE)
        elif args.command == "run":
            tester.send(MODULE_CHASSIS, CMD_CHASSIS_RUN, tester.f32(args.vx, args.vy, args.wz))
        elif args.command == "stop":
            tester.send(MODULE_CHASSIS, CMD_CHASSIS_STOP)
        elif args.command == "brake":
            tester.send(MODULE_CHASSIS, CMD_CHASSIS_BRAKE)
        elif args.command == "rotate":
            tester.send(MODULE_CHASSIS, CMD_CHASSIS_ROTATE_TO, tester.f32(args.angle, args.wz or args.speed))
        elif args.command == "translate":
            tester.send(MODULE_CHASSIS, CMD_CHASSIS_TRANSLATE_REL, tester.f32(args.dx, args.dy, args.speed))
        elif args.command == "arm_pose":
            tester.send(MODULE_ARM, CMD_ARM_GET_POSE)
        elif args.command == "arm_fold":
            tester.send(MODULE_ARM, CMD_ARM_FOLD_HOME)
        elif args.command == "arm_stop":
            tester.send(MODULE_ARM, CMD_ARM_STOP_HOLD)
        elif args.command == "arm_target":
            tester.send(MODULE_ARM, CMD_ARM_TARGET_XYZ_DIR, tester.f32(args.x, args.y, args.z, args.dir))
        elif args.command == "pump_state":
            tester.send(MODULE_PUMP, CMD_PUMP_GET_STATE)
        elif args.command == "pump_suck":
            tester.send(MODULE_PUMP, CMD_PUMP_SUCK)
        elif args.command == "pump_release":
            tester.send(MODULE_PUMP, CMD_PUMP_RELEASE)

        tester.print_rx(args.listen)
    finally:
        tester.close()


if __name__ == "__main__":
    main()
