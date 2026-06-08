from __future__ import annotations

from collections import deque
from pathlib import Path
import copy
import struct
import threading
import time

try:
    import serial
except ImportError:  # pragma: no cover - depends on host environment
    serial = None

try:
    import yaml
except ImportError:  # pragma: no cover - optional dependency
    yaml = None


DEFAULT_CONFIG = {
    "serial": {
        "port": "COM8",
        "baudrate": 115200,
        "timeout_s": 0.03,
        "read_sleep_s": 0.01,
        "print_rx_to_terminal": True,
        "rx_print_prefix": "[MCU] ",
    },
    "protocol": {
        "header": [0xAA, 0x55],
        "tail": [0x0D, 0x0A],
        "version": 0x01,
        "max_rx_payload_len": 32,
        "max_tx_payload_len": 64,
    },
    "modules": {
        "system": 0x01,
        "chassis": 0x02,
        "arm": 0x03,
        "pump": 0x04,
    },
    "commands": {
        "system": {"hello": 0x01, "reset": 0x02, "estop": 0x03, "get_status": 0x04},
        "chassis": {
            "run": 0x10,
            "stop": 0x11,
            "brake": 0x12,
            "rotate_to": 0x13,
            "translate_rel": 0x14,
            "get_state": 0x15,
        },
        "arm": {"target_xyz_dir": 0x20, "fold_home": 0x21, "stop_hold": 0x22, "get_pose": 0x23},
        "pump": {"suck": 0x30, "release": 0x31, "get_state": 0x32},
    },
    "responses": {
        "ack": 0x80,
        "nack": 0x81,
        "status": 0x82,
        "event": 0x83,
        "arm_pose": 0x84,
        "chassis_state": 0x85,
        "pump_state": 0x86,
    },
    "payload_formats": {
        "chassis_run": "<fff",
        "chassis_rotate_to": "<ff",
        "chassis_translate_rel": "<fff",
        "arm_target_xyz_dir": "<ffff",
        "ack": "<BBBBB",
        "event": "<BBBBB",
        "status": "<BBBBBBBBIf",
        "chassis_state": "<BBfffff",
        "arm_pose": "<BBBBffffffff",
        "pump_state": "<BBBBfBB",
    },
    "controller": {
        "auto_start_reader": True,
        "wait_timeout_s": 1.0,
        "event_cache_len": 100,
        "frame_cache_len": 100,
    },
}


def deep_merge(base, override):
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def config_get(config, dotted_key, default=None):
    current = config
    for key in dotted_key.split("."):
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def load_config(path=None):
    config = copy.deepcopy(DEFAULT_CONFIG)
    if path is None:
        path = Path(__file__).with_name("communication.yml")
    path = Path(path)
    if path.exists() and yaml is not None:
        with path.open("r", encoding="utf-8") as fp:
            loaded = yaml.safe_load(fp) or {}
        config = deep_merge(config, loaded)
    return config


def clamp(value, min_value, max_value):
    value = float(value)
    if min_value is not None and value < min_value:
        return float(min_value)
    if max_value is not None and value > max_value:
        return float(max_value)
    return value


class NanoController:
    def __init__(self, port=None, config_path=None, auto_open=False):
        self.config = load_config(config_path)
        self.port = port or config_get(self.config, "serial.port", "COM8")
        self.baudrate = int(config_get(self.config, "serial.baudrate", 115200))
        self.timeout_s = float(config_get(self.config, "serial.timeout_s", 0.03))
        self.read_sleep_s = float(config_get(self.config, "serial.read_sleep_s", 0.01))
        self.print_rx = bool(config_get(self.config, "serial.print_rx_to_terminal", True))
        self.rx_prefix = str(config_get(self.config, "serial.rx_print_prefix", "[MCU] "))
        self.header = bytes(config_get(self.config, "protocol.header", [0xAA, 0x55]))
        self.tail = bytes(config_get(self.config, "protocol.tail", [0x0D, 0x0A]))
        self.version = int(config_get(self.config, "protocol.version", 0x01)) & 0xFF
        self.max_rx_payload_len = int(config_get(self.config, "protocol.max_rx_payload_len", 32))
        self.seq = 0
        self.serial = None
        self.rx_buffer = bytearray()
        self.reader_running = False
        self.reader_thread = None
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.frames = deque(maxlen=int(config_get(self.config, "controller.frame_cache_len", 100)))
        self.events = deque(maxlen=int(config_get(self.config, "controller.event_cache_len", 100)))
        self.acks = {}
        self.nacks = {}
        self.status = {}
        self.chassis_state = {}
        self.arm_pose = {}
        self.pump_state = {}
        if auto_open:
            self.open()

    def open(self):
        if serial is None:
            raise RuntimeError("pyserial is required: pip install pyserial")
        if self.serial and self.serial.is_open:
            return
        self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)
        if bool(config_get(self.config, "controller.auto_start_reader", True)):
            self.start_reader()

    def close(self):
        self.stop_reader()
        if self.serial:
            self.serial.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def start_reader(self):
        if self.reader_running:
            return
        self.reader_running = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

    def stop_reader(self):
        self.reader_running = False
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.5)
        self.reader_thread = None

    def _reader_loop(self):
        while self.reader_running:
            try:
                data = self.serial.read(256) if self.serial else b""
            except Exception as exc:
                with self.condition:
                    self.events.append({"type": "read_error", "error": str(exc), "time": time.time()})
                    self.condition.notify_all()
                break
            if data:
                for frame in self.feed(data):
                    self._handle_frame(frame)
            else:
                time.sleep(self.read_sleep_s)

    def checksum(self, seq, module, cmd, payload):
        return (self.version + seq + module + cmd + len(payload) + sum(payload)) & 0xFF

    def next_seq(self):
        with self.lock:
            self.seq = (self.seq + 1) & 0xFF
            return self.seq

    def build_frame(self, module, cmd, payload=b"", seq=None):
        module = int(module) & 0xFF
        cmd = int(cmd) & 0xFF
        payload = bytes(payload or b"")
        if len(payload) > 255:
            raise ValueError("payload too long for one-byte length")
        if seq is None:
            seq = self.next_seq()
        body = bytes([self.version, seq, module, cmd, len(payload)]) + payload
        return seq, self.header + body + bytes([self.checksum(seq, module, cmd, payload)]) + self.tail

    def send_frame(self, module, cmd, payload=b"", wait_ack=False, timeout_s=None):
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("serial port is not open")
        seq, frame = self.build_frame(module, cmd, payload)
        self.serial.write(frame)
        self.serial.flush()
        if wait_ack:
            return self.wait_for_ack(seq, timeout_s=timeout_s)
        return seq

    def feed(self, data):
        parsed = []
        self.rx_buffer.extend(data)
        while True:
            head = self.rx_buffer.find(self.header)
            if head < 0:
                self.rx_buffer.clear()
                break
            if head > 0:
                del self.rx_buffer[:head]
            if len(self.rx_buffer) < 9:
                break
            payload_len = self.rx_buffer[6]
            frame_len = 2 + 5 + payload_len + 1 + 2
            if len(self.rx_buffer) < frame_len:
                break
            raw = bytes(self.rx_buffer[:frame_len])
            del self.rx_buffer[:frame_len]
            if raw[-2:] != self.tail:
                continue
            ver, seq, module, cmd, length = raw[2], raw[3], raw[4], raw[5], raw[6]
            payload = raw[7:7 + length]
            check = raw[7 + length]
            if ver == self.version and check == self.checksum(seq, module, cmd, payload):
                parsed.append({"seq": seq, "module": module, "cmd": cmd, "payload": payload, "raw": raw})
        return parsed

    def _handle_frame(self, frame):
        parsed = self.parse_frame(frame)
        with self.condition:
            self.frames.append(parsed)
            kind = parsed.get("kind")
            source_seq = parsed.get("source_seq", parsed.get("seq"))
            if kind == "ack":
                self.acks[source_seq] = parsed
            elif kind == "nack":
                self.nacks[source_seq] = parsed
            elif kind == "event":
                self.events.append(parsed)
            elif kind == "status":
                self.status = parsed
            elif kind == "chassis_state":
                self.chassis_state = parsed
            elif kind == "arm_pose":
                self.arm_pose = parsed
            elif kind == "pump_state":
                self.pump_state = parsed
            if self.print_rx:
                print(self.rx_prefix + self.format_frame(parsed))
            self.condition.notify_all()

    def parse_frame(self, frame):
        payload = frame["payload"]
        response = self.config["responses"]
        result = dict(frame)
        cmd = frame["cmd"]
        if cmd == response["ack"] and len(payload) >= 5:
            source_seq, module, source_cmd, err, state = struct.unpack_from("<BBBBB", payload)
            result.update(kind="ack", source_seq=source_seq, source_module=module,
                          source_cmd=source_cmd, err=err, state=state)
        elif cmd == response["nack"] and len(payload) >= 5:
            source_seq, module, source_cmd, err, state = struct.unpack_from("<BBBBB", payload)
            result.update(kind="nack", source_seq=source_seq, source_module=module,
                          source_cmd=source_cmd, err=err, state=state)
        elif cmd == response["event"] and len(payload) >= 5:
            source_seq, module, event, state, result_code = struct.unpack_from("<BBBBB", payload)
            result.update(kind="event", source_seq=source_seq, event=event,
                          state=state, result=result_code)
        elif cmd == response["status"] and len(payload) >= 16:
            fields = struct.unpack_from("<BBBBBBBBIf", payload)
            result.update(kind="status", system_state=fields[0], chassis_state=fields[1],
                          arm_state=fields[2], pump_state=fields[3], movement=fields[4],
                          pump_on=fields[5], sucked=fields[6], arm_planner_active=fields[7],
                          tick_ms=fields[8], yaw_deg=fields[9])
        elif cmd == response["chassis_state"] and len(payload) >= 22:
            fields = struct.unpack_from("<BBfffff", payload)
            result.update(kind="chassis_state", chassis_state=fields[0], movement=fields[1],
                          yaw_deg=fields[2], vx=fields[3], vy=fields[4],
                          wz=fields[5], distance_cm=fields[6])
        elif cmd == response["arm_pose"] and len(payload) >= 36:
            fields = struct.unpack_from("<BBBBffffffff", payload)
            result.update(kind="arm_pose", arm_state=fields[0], planner_active=fields[2],
                          x=fields[4], y=fields[5], z=fields[6], suction_dir_deg=fields[7],
                          th0=fields[8], th1=fields[9], th2=fields[10], th3=fields[11])
        elif cmd == response["pump_state"] and len(payload) >= 10:
            fields = struct.unpack_from("<BBBBfBB", payload)
            result.update(kind="pump_state", pump_state=fields[0], pump_on=fields[1],
                          sucked=fields[2], feedback_hz=fields[4],
                          suck_strength=fields[5])
        else:
            result.update(kind="raw")
        return result

    def format_frame(self, frame):
        kind = frame.get("kind", "raw")
        if kind in ("ack", "nack"):
            return f"{kind.upper()} seq={frame.get('source_seq')} module=0x{frame.get('source_module', 0):02X} cmd=0x{frame.get('source_cmd', 0):02X} err=0x{frame.get('err', 0):02X} state={frame.get('state')}"
        if kind == "event":
            return f"EVENT seq={frame.get('source_seq')} event=0x{frame.get('event', 0):02X} state={frame.get('state')} result=0x{frame.get('result', 0):02X}"
        if kind == "status":
            return f"STATUS sys={frame.get('system_state')} chassis={frame.get('chassis_state')} arm={frame.get('arm_state')} pump={frame.get('pump_state')} yaw={frame.get('yaw_deg'):.2f}"
        if kind == "chassis_state":
            return f"CHASSIS yaw={frame.get('yaw_deg'):.2f} vx={frame.get('vx'):.2f} vy={frame.get('vy'):.2f} wz={frame.get('wz'):.2f} dist_cm={frame.get('distance_cm'):.2f}"
        if kind == "arm_pose":
            return f"ARM x={frame.get('x'):.2f} y={frame.get('y'):.2f} z={frame.get('z'):.2f} dir={frame.get('suction_dir_deg'):.2f}"
        if kind == "pump_state":
            return f"PUMP state={frame.get('pump_state')} on={frame.get('pump_on')} sucked={frame.get('sucked')} freq={frame.get('feedback_hz'):.2f}"
        return f"RAW module=0x{frame.get('module', 0):02X} cmd=0x{frame.get('cmd', 0):02X} len={len(frame.get('payload', b''))}"

    def wait_for_ack(self, seq, timeout_s=None):
        timeout_s = float(timeout_s or config_get(self.config, "controller.wait_timeout_s", 1.0))
        end_time = time.time() + timeout_s
        with self.condition:
            while time.time() < end_time:
                if seq in self.acks:
                    return self.acks.pop(seq)
                if seq in self.nacks:
                    return self.nacks.pop(seq)
                self.condition.wait(timeout=max(0.0, end_time - time.time()))
        return None

    def wait_for_event(self, event_code=None, source_seq=None, timeout_s=None):
        timeout_s = float(timeout_s or config_get(self.config, "controller.wait_timeout_s", 1.0))
        end_time = time.time() + timeout_s
        with self.condition:
            while time.time() < end_time:
                for event in list(self.events):
                    if event.get("kind") != "event":
                        continue
                    if event_code is not None and event.get("event") != event_code:
                        continue
                    if source_seq is not None and event.get("source_seq") != source_seq:
                        continue
                    return event
                self.condition.wait(timeout=max(0.0, end_time - time.time()))
        return None

    def _module(self, name):
        return int(config_get(self.config, f"modules.{name}"))

    def _cmd(self, module, name):
        return int(config_get(self.config, f"commands.{module}.{name}"))

    def _send(self, module_name, cmd_name, payload=b"", wait_ack=False):
        return self.send_frame(self._module(module_name), self._cmd(module_name, cmd_name),
                               payload, wait_ack=wait_ack)

    def system_hello(self, wait_ack=True):
        return self._send("system", "hello", wait_ack=wait_ack)

    def system_status(self):
        return self._send("system", "get_status")

    def system_reset(self, wait_ack=True):
        return self._send("system", "reset", wait_ack=wait_ack)

    def system_estop(self, wait_ack=True):
        return self._send("system", "estop", wait_ack=wait_ack)

    def chassis_run(self, vx, vy, wz=0.0, wait_ack=False):
        limits = config_get(self.config, "limits.chassis", {})
        vx = clamp(vx, limits.get("run_vx_input_min", -100.0), limits.get("run_vx_input_max", 100.0))
        vy = clamp(vy, limits.get("run_vy_input_min", -100.0), limits.get("run_vy_input_max", 100.0))
        wz = clamp(wz, limits.get("run_wz_input_min", -100.0), limits.get("run_wz_input_max", 100.0))
        return self._send("chassis", "run", struct.pack("<fff", vx, vy, wz), wait_ack=wait_ack)

    def chassis_stop(self, wait_ack=True):
        return self._send("chassis", "stop", wait_ack=wait_ack)

    def chassis_brake(self, wait_ack=True):
        return self._send("chassis", "brake", wait_ack=wait_ack)

    def chassis_rotate_to(self, angle_deg, max_wz, wait_ack=True):
        limits = config_get(self.config, "limits.chassis", {})
        max_wz = clamp(max_wz, limits.get("rotate_wz_input_min", 1.0), limits.get("rotate_wz_input_max", 100.0))
        return self._send("chassis", "rotate_to", struct.pack("<ff", float(angle_deg), max_wz), wait_ack=wait_ack)

    def chassis_translate_rel(self, dx_cm, dy_cm, speed, wait_ack=True):
        limits = config_get(self.config, "limits.chassis", {})
        dx_cm = clamp(dx_cm, limits.get("translate_dx_cm_min", -1000.0), limits.get("translate_dx_cm_max", 1000.0))
        dy_cm = clamp(dy_cm, limits.get("translate_dy_cm_min", -1000.0), limits.get("translate_dy_cm_max", 1000.0))
        speed = clamp(speed, limits.get("translate_speed_input_min", 1.0), limits.get("translate_speed_input_max", 100.0))
        return self._send("chassis", "translate_rel", struct.pack("<fff", dx_cm, dy_cm, speed), wait_ack=wait_ack)

    def chassis_get_state(self):
        return self._send("chassis", "get_state")

    def arm_target(self, x_cm, y_cm, z_cm, suction_dir_deg, wait_ack=True):
        payload = struct.pack("<ffff", float(x_cm), float(y_cm), float(z_cm), float(suction_dir_deg))
        return self._send("arm", "target_xyz_dir", payload, wait_ack=wait_ack)

    def arm_fold_home(self, wait_ack=True):
        return self._send("arm", "fold_home", wait_ack=wait_ack)

    def arm_stop_hold(self, wait_ack=True):
        return self._send("arm", "stop_hold", wait_ack=wait_ack)

    def arm_get_pose(self):
        return self._send("arm", "get_pose")

    def pump_suck(self, wait_ack=True):
        return self._send("pump", "suck", wait_ack=wait_ack)

    def pump_release(self, wait_ack=True):
        return self._send("pump", "release", wait_ack=wait_ack)

    def pump_get_state(self):
        return self._send("pump", "get_state")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="NANO controller quick test")
    parser.add_argument("--port", default=None)
    parser.add_argument("--config", default=None)
    parser.add_argument("command", choices=["hello", "status", "brake", "stop"])
    args = parser.parse_args()

    with NanoController(port=args.port, config_path=args.config, auto_open=True) as ctl:
        if args.command == "hello":
            ctl.system_hello()
        elif args.command == "status":
            ctl.system_status()
        elif args.command == "brake":
            ctl.chassis_brake()
        elif args.command == "stop":
            ctl.chassis_stop()
        time.sleep(0.5)
