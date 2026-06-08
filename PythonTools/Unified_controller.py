from pathlib import Path
import copy
import struct
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


DEFAULT_CONFIG = {
    "serial": {
        "baudrate": 115200,
        "timeout_s": 0.1,
        "read_sleep_s": 0.01,
        "encoding": "utf-8",
        "print_rx_to_terminal": True,
        "rx_print_prefix": "[MCU] ",
    },
    "protocol": {
        "header": [0xAA, 0x55],
        "tail": [0x0D, 0x0A],
        "version": 0x02,
        "max_payload_len": 192,
    },
    "commands": {
        "hello": 0x01,
        "get_status": 0x02,
        "set_feedback": 0x03,
        "set_yaw": 0x04,
        "chassis_brake": 0x10,
        "chassis_run": 0x11,
        "arm_stop": 0x20,
        "arm_target": 0x21,
        "arm_fold": 0x22,
    },
    "responses": {
        "ack": 0x80,
        "error": 0x81,
        "status": 0x82,
        "text": 0x83,
        "arm_event": 0x84,
    },
    "payload_formats": {
        "set_feedback": "<H",
        "set_yaw": "<f",
        "chassis_run": "<fff",
        "arm_target": "<ffff",
        "status": "<BBBBfffffffI",
        "arm_event": "<BBBBBBBBffffffffI",
    },
}


COMMAND_NAMES = {
    0x01: "hello",
    0x02: "get_status",
    0x03: "set_feedback",
    0x04: "set_yaw",
    0x10: "chassis_brake",
    0x11: "chassis_run",
    0x20: "arm_stop",
    0x21: "arm_target",
    0x22: "arm_fold",
}


RESPONSE_NAMES = {
    0x80: "ack",
    0x81: "error",
    0x82: "status",
    0x83: "text",
    0x84: "arm_event",
}


ERROR_NAMES = {
    0x00: "NONE",
    0x01: "BAD_CMD",
    0x02: "BAD_LEN",
    0x03: "BAD_CHECK",
    0x04: "BUSY",
    0x05: "TIMEOUT",
    0x06: "REJECTED",
}


ARM_EVENT_NAMES = {
    0x01: "ACCEPTED",
    0x02: "COMPLETED",
    0x03: "REJECTED",
    0x04: "STOPPED",
    0x05: "PHASE_ERROR",
}


ARM_STATE_NAMES = {
    0x00: "IDLE",
    0x01: "RUNNING",
    0x02: "DONE",
    0x03: "REJECTED",
    0x04: "STOPPED",
    0x05: "ERROR",
}


ARM_TERMINAL_EVENTS = {0x02, 0x03, 0x04, 0x05}


def config_get(config, dotted_key, default=None):
    current = config
    for key in dotted_key.split("."):
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def _lookup_name(config, table_key, value, fallback_table, default_prefix="UNKNOWN"):
    value = int(value) & 0xFF
    table = config_get(config, table_key, {})
    if isinstance(table, dict):
        candidate_keys = (
            value,
            f"0x{value:02X}",
            f"0x{value:02x}",
            str(value),
        )
        for key in candidate_keys:
            if key in table:
                return str(table[key])
    return fallback_table.get(value, f"{default_prefix}_0x{value:02X}")


def _deep_merge(base, override):
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _strip_comment(line):
    in_single = False
    in_double = False
    for index, char in enumerate(line):
        if char == "'" and not in_double:
            in_single = not in_single
        elif char == '"' and not in_single:
            in_double = not in_double
        elif char == "#" and not in_single and not in_double:
            return line[:index]
    return line


def _parse_scalar(value):
    value = value.strip()
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(item) for item in inner.split(",")]
    if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]

    lowered = value.lower()
    if lowered in ("true", "yes", "on"):
        return True
    if lowered in ("false", "no", "off"):
        return False
    if lowered == "null" or value == "~":
        return None

    try:
        return int(value, 0)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _minimal_yaml_load(text):
    root = {}
    stack = [(-1, root)]

    for raw_line in text.splitlines():
        line = _strip_comment(raw_line).rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        item = line.strip()
        if ":" not in item:
            continue

        key, value = item.split(":", 1)
        key = key.strip()
        value = value.strip()

        while indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]

        if value == "":
            node = {}
            parent[key] = node
            stack.append((indent, node))
        else:
            parent[key] = _parse_scalar(value)

    return root


def load_unified_communication_config(config_path=None):
    if config_path is None:
        config_path = Path(__file__).with_name("Unified_communication.yml")
    config_path = Path(config_path)
    if not config_path.exists():
        return copy.deepcopy(DEFAULT_CONFIG)

    text = config_path.read_text(encoding="utf-8")
    try:
        import yaml
        user_config = yaml.safe_load(text) or {}
    except Exception:
        user_config = _minimal_yaml_load(text)

    return _deep_merge(DEFAULT_CONFIG, user_config)


class UnifiedCommunicationController:
    def __init__(
            self,
            config=None,
            config_path=None,
            on_frame=None,
            on_status=None,
            on_text=None,
            on_ack=None,
            on_error_frame=None,
            on_arm_event=None,
            on_raw_text=None,
            on_error=None,
            print_received=None,
            rx_print_prefix=None,
    ):
        self.config = config if config is not None else load_unified_communication_config(config_path)
        self.on_frame = on_frame
        self.on_status = on_status
        self.on_text = on_text
        self.on_ack = on_ack
        self.on_error_frame = on_error_frame
        self.on_arm_event = on_arm_event
        self.on_raw_text = on_raw_text
        self.on_error = on_error
        self.print_received = (
            config_get(self.config, "serial.print_rx_to_terminal", True)
            if print_received is None else bool(print_received)
        )
        self.rx_print_prefix = (
            config_get(self.config, "serial.rx_print_prefix", "[MCU] ")
            if rx_print_prefix is None else str(rx_print_prefix)
        )
        self.serial_port = None
        self._seq = 0
        self._receiving = False
        self._receiver_thread = None
        self._state_lock = threading.RLock()
        self._arm_condition = threading.Condition(self._state_lock)
        self.last_frame = None
        self.last_ack = None
        self.last_error_frame = None
        self.last_status = None
        self.last_text = None
        self.last_arm_event = None
        self.received_texts = []
        self.state = self._new_state()

    @staticmethod
    def _new_state():
        return {
            "connected": False,
            "last_frame": None,
            "last_ack": None,
            "last_error_frame": None,
            "last_status": None,
            "last_text": None,
            "last_raw_text": None,
            "last_arm_event": None,
            "texts": [],
            "raw_texts": [],
            "counters": {
                "frame": 0,
                "ack": 0,
                "error": 0,
                "status": 0,
                "text": 0,
                "raw_text": 0,
                "arm_event": 0,
                "unknown": 0,
            },
            "arm": {
                "state": 0,
                "state_name": "IDLE",
                "last_event": None,
                "last_event_name": None,
                "source_cmd": None,
                "source_cmd_name": None,
                "error": 0,
                "error_name": "NONE",
                "phase_index": 0,
                "phase_count": 0,
                "planner_active": False,
                "target": {
                    "x_cm": 0.0,
                    "y_cm": 0.0,
                    "z_cm": 0.0,
                    "suc_dir_deg": 0.0,
                },
                "joint": {
                    "th0_deg": 0.0,
                    "th1_deg": 0.0,
                    "th2_deg": 0.0,
                    "th3_deg": 0.0,
                },
                "timestamp_ms": 0,
            },
        }

    def get_state_snapshot(self):
        with self._state_lock:
            return copy.deepcopy(self.state)

    def command_name(self, value):
        return _lookup_name(self.config, "command_names", value, COMMAND_NAMES, "CMD")

    def response_name(self, value):
        return _lookup_name(self.config, "response_names", value, RESPONSE_NAMES, "RESP")

    def error_name(self, value):
        return _lookup_name(self.config, "error_names", value, ERROR_NAMES, "ERR")

    def arm_event_name(self, value):
        return _lookup_name(self.config, "arm_event_names", value, ARM_EVENT_NAMES, "ARM_EVENT")

    def arm_state_name(self, value):
        return _lookup_name(self.config, "arm_state_names", value, ARM_STATE_NAMES, "ARM_STATE")

    @staticmethod
    def list_ports():
        if serial is None:
            raise RuntimeError("缺少 pyserial，请先安装 pyserial 后再扫描串口")
        return [port.device for port in serial.tools.list_ports.comports()]

    @property
    def is_open(self):
        return self.serial_port is not None and self.serial_port.is_open

    def connect(self, port, baudrate=None, timeout=None, start_receiver=True):
        if serial is None:
            raise RuntimeError("缺少 pyserial，请先安装 pyserial 后再连接串口")
        baudrate = baudrate if baudrate is not None else config_get(self.config, "serial.baudrate", 115200)
        timeout = timeout if timeout is not None else config_get(self.config, "serial.timeout_s", 0.1)
        self.serial_port = serial.Serial(port, baudrate, timeout=timeout)
        with self._state_lock:
            self.state["connected"] = True
        if start_receiver:
            self.start_receiver()
        return self.serial_port

    def disconnect(self):
        self.stop_receiver()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        with self._state_lock:
            self.state["connected"] = False

    def close(self):
        self.disconnect()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def start_receiver(self):
        if self._receiving:
            return
        self._receiving = True
        self._receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receiver_thread.start()

    def stop_receiver(self):
        self._receiving = False
        if self._receiver_thread and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=0.3)
        self._receiver_thread = None

    def command(self, name):
        value = config_get(self.config, f"commands.{name}")
        if value is None:
            raise KeyError(f"未知命令名: {name}")
        return int(value)

    def response(self, name):
        value = config_get(self.config, f"responses.{name}")
        if value is None:
            raise KeyError(f"未知响应名: {name}")
        return int(value)

    def build_packet(self, cmd_type, payload=b"", seq=None):
        header = bytes(config_get(self.config, "protocol.header", [0xAA, 0x55]))
        tail = bytes(config_get(self.config, "protocol.tail", [0x0D, 0x0A]))
        version = int(config_get(self.config, "protocol.version", 0x02))
        max_payload_len = int(config_get(self.config, "protocol.max_payload_len", 192))
        payload = bytes(payload)
        if len(payload) > max_payload_len:
            raise ValueError(f"payload 过长: {len(payload)} > {max_payload_len}")

        if seq is None:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFF
        seq = int(seq) & 0xFF
        cmd_type = int(cmd_type) & 0xFF
        payload_len = len(payload)
        checksum = (version + seq + cmd_type + payload_len + sum(payload)) & 0xFF
        return header + bytes([version, seq, cmd_type, payload_len]) + payload + bytes([checksum]) + tail

    def send_packet(self, cmd_type, payload=b""):
        if not self.is_open:
            raise RuntimeError("串口未连接")
        packet = self.build_packet(cmd_type, payload)
        self.serial_port.write(packet)
        return packet

    def send_hello(self):
        return self.send_packet(self.command("hello"))

    def get_status(self):
        return self.send_packet(self.command("get_status"))

    def set_feedback(self, interval_ms):
        payload_format = config_get(self.config, "payload_formats.set_feedback", "<H")
        payload = struct.pack(payload_format, int(interval_ms))
        return self.send_packet(self.command("set_feedback"), payload)

    def set_yaw(self, yaw_deg):
        payload_format = config_get(self.config, "payload_formats.set_yaw", "<f")
        payload = struct.pack(payload_format, float(yaw_deg))
        return self.send_packet(self.command("set_yaw"), payload)

    def chassis_brake(self):
        return self.send_packet(self.command("chassis_brake"))

    def chassis_run(self, vx_cm_s, vy_cm_s, wz_deg_s):
        payload_format = config_get(self.config, "payload_formats.chassis_run", "<fff")
        payload = struct.pack(payload_format, float(vx_cm_s), float(vy_cm_s), float(wz_deg_s))
        return self.send_packet(self.command("chassis_run"), payload)

    def arm_stop(self):
        return self.send_packet(self.command("arm_stop"))

    def arm_target(self, world_x_cm, world_y_cm, world_z_cm, suc_dir_deg):
        payload_format = config_get(self.config, "payload_formats.arm_target", "<ffff")
        payload = struct.pack(
            payload_format,
            float(world_x_cm),
            float(world_y_cm),
            float(world_z_cm),
            float(suc_dir_deg),
        )
        return self.send_packet(self.command("arm_target"), payload)

    def arm_fold(self):
        return self.send_packet(self.command("arm_fold"))

    def parse_frame(self, frame):
        frame = bytes(frame)
        header = bytes(config_get(self.config, "protocol.header", [0xAA, 0x55]))
        tail = bytes(config_get(self.config, "protocol.tail", [0x0D, 0x0A]))
        if len(frame) < 9:
            raise ValueError("帧长度过短")
        if frame[:2] != header or frame[-2:] != tail:
            raise ValueError("帧头或帧尾错误")

        version = frame[2]
        seq = frame[3]
        cmd = frame[4]
        payload_len = frame[5]
        payload = frame[6:6 + payload_len]
        checksum = frame[6 + payload_len]
        expected = (version + seq + cmd + payload_len + sum(payload)) & 0xFF
        if checksum != expected:
            raise ValueError(f"校验失败: got 0x{checksum:02X}, expected 0x{expected:02X}")
        return {"version": version, "seq": seq, "cmd": cmd, "payload": payload, "checksum": checksum}

    def parse_status_payload(self, payload):
        payload_format = config_get(self.config, "payload_formats.status", "<BBBBfffffffI")
        fields = struct.unpack(payload_format, bytes(payload))
        return {
            "chassis_mode": fields[0],
            "chassis_motion_active": fields[1],
            "arm_cmd_type": fields[2],
            "arm_new_cmd_flag": fields[3],
            "yaw_deg": fields[4],
            "target_vx_cm_s": fields[5],
            "target_vy_cm_s": fields[6],
            "target_wz_deg_s": fields[7],
            "arm_world_x_cm": fields[8],
            "arm_world_y_cm": fields[9],
            "arm_world_z_cm": fields[10],
            "timestamp_ms": fields[11],
        }

    def parse_arm_event_payload(self, payload):
        payload_format = config_get(self.config, "payload_formats.arm_event", "<BBBBBBBBffffffffI")
        fields = struct.unpack(payload_format, bytes(payload))
        event = fields[0]
        source_cmd = fields[1]
        state = fields[2]
        error = fields[3]
        return {
            "event": event,
            "event_name": self.arm_event_name(event),
            "source_cmd": source_cmd,
            "source_cmd_name": self.command_name(source_cmd),
            "state": state,
            "state_name": self.arm_state_name(state),
            "error": error,
            "error_name": self.error_name(error),
            "phase_index": fields[4],
            "phase_count": fields[5],
            "planner_active": bool(fields[6]),
            "reserved": fields[7],
            "target_x_cm": fields[8],
            "target_y_cm": fields[9],
            "target_z_cm": fields[10],
            "suc_dir_deg": fields[11],
            "th0_deg": fields[12],
            "th1_deg": fields[13],
            "th2_deg": fields[14],
            "th3_deg": fields[15],
            "timestamp_ms": fields[16],
            "target": {
                "x_cm": fields[8],
                "y_cm": fields[9],
                "z_cm": fields[10],
                "suc_dir_deg": fields[11],
            },
            "joint": {
                "th0_deg": fields[12],
                "th1_deg": fields[13],
                "th2_deg": fields[14],
                "th3_deg": fields[15],
            },
        }

    def _store_frame(self, parsed):
        frame_info = {
            "version": parsed["version"],
            "seq": parsed["seq"],
            "cmd": parsed["cmd"],
            "cmd_name": self.response_name(parsed["cmd"]),
            "payload": parsed["payload"],
            "checksum": parsed["checksum"],
            "timestamp_local_s": time.time(),
        }
        with self._state_lock:
            self.last_frame = frame_info
            self.state["last_frame"] = frame_info
            self.state["counters"]["frame"] += 1

    def _store_ack(self, info):
        with self._state_lock:
            self.last_ack = info
            self.state["last_ack"] = info
            self.state["counters"]["ack"] += 1

    def _store_error_frame(self, info):
        with self._state_lock:
            self.last_error_frame = info
            self.state["last_error_frame"] = info
            self.state["counters"]["error"] += 1

    def _store_status(self, status):
        with self._state_lock:
            self.last_status = status
            self.state["last_status"] = status
            self.state["counters"]["status"] += 1

    def _store_text(self, text):
        with self._state_lock:
            self.last_text = text
            self.received_texts.append(text)
            max_texts = int(config_get(self.config, "serial.max_cached_text_lines", 100))
            if max_texts > 0:
                del self.received_texts[:-max_texts]
            self.state["last_text"] = text
            self.state["texts"] = list(self.received_texts)
            self.state["counters"]["text"] += 1

    def _store_raw_text(self, text):
        with self._state_lock:
            self.state["last_raw_text"] = text
            raw_texts = self.state["raw_texts"]
            raw_texts.append(text)
            max_texts = int(config_get(self.config, "serial.max_cached_text_lines", 100))
            if max_texts > 0:
                del raw_texts[:-max_texts]
            self.state["counters"]["raw_text"] += 1

    def _store_arm_event(self, event):
        with self._arm_condition:
            self.last_arm_event = event
            self.state["last_arm_event"] = event
            self.state["arm"] = {
                "state": event["state"],
                "state_name": event["state_name"],
                "last_event": event["event"],
                "last_event_name": event["event_name"],
                "source_cmd": event["source_cmd"],
                "source_cmd_name": event["source_cmd_name"],
                "error": event["error"],
                "error_name": event["error_name"],
                "phase_index": event["phase_index"],
                "phase_count": event["phase_count"],
                "planner_active": event["planner_active"],
                "target": event["target"],
                "joint": event["joint"],
                "timestamp_ms": event["timestamp_ms"],
            }
            self.state["counters"]["arm_event"] += 1
            self._arm_condition.notify_all()

    def wait_for_arm_terminal_event(self, timeout_s=None, min_event_count=None):
        deadline = None if timeout_s is None else time.monotonic() + float(timeout_s)
        with self._arm_condition:
            if min_event_count is None:
                min_event_count = self.state["counters"]["arm_event"]
            while True:
                event = self.last_arm_event
                event_count = self.state["counters"]["arm_event"]
                if event is not None and event_count > min_event_count and event["event"] in ARM_TERMINAL_EVENTS:
                    return event
                if deadline is None:
                    self._arm_condition.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._arm_condition.wait(remaining)

    def _handle_frame(self, parsed):
        cmd = parsed["cmd"]
        payload = parsed["payload"]
        self._store_frame(parsed)
        if self.on_frame:
            self.on_frame(parsed)

        if cmd == self.response("ack") and len(payload) >= 2:
            info = {
                "source_cmd": payload[0],
                "source_cmd_name": self.command_name(payload[0]),
                "error": payload[1],
                "error_name": self.error_name(payload[1]),
            }
            self._store_ack(info)
            if self.print_received:
                print(f"{self.rx_print_prefix}ACK source={info['source_cmd_name']}(0x{payload[0]:02X}) err={info['error_name']}", flush=True)
            if self.on_ack:
                self.on_ack(info)
        elif cmd == self.response("error") and len(payload) >= 2:
            info = {
                "source_cmd": payload[0],
                "source_cmd_name": self.command_name(payload[0]),
                "error": payload[1],
                "error_name": self.error_name(payload[1]),
            }
            self._store_error_frame(info)
            if self.print_received:
                print(f"{self.rx_print_prefix}ERR source={info['source_cmd_name']}(0x{payload[0]:02X}) err={info['error_name']}", flush=True)
            if self.on_error_frame:
                self.on_error_frame(info)
        elif cmd == self.response("status"):
            status = self.parse_status_payload(payload)
            self._store_status(status)
            if self.print_received:
                print(f"{self.rx_print_prefix}STATUS {status}", flush=True)
            if self.on_status:
                self.on_status(status)
        elif cmd == self.response("text"):
            encoding = config_get(self.config, "serial.encoding", "utf-8")
            text = bytes(payload).decode(encoding, errors="ignore").strip()
            self._store_text(text)
            if self.print_received:
                print(f"{self.rx_print_prefix}{text}", flush=True)
            if self.on_text:
                self.on_text(text)
        elif cmd == self.response("arm_event"):
            event = self.parse_arm_event_payload(payload)
            self._store_arm_event(event)
            if self.print_received:
                print(
                    f"{self.rx_print_prefix}ARM_EVENT {event['event_name']} "
                    f"source={event['source_cmd_name']}(0x{event['source_cmd']:02X}) "
                    f"state={event['state_name']} err={event['error_name']} "
                    f"phase={event['phase_index']}/{event['phase_count']} "
                    f"active={int(event['planner_active'])} "
                    f"target=({event['target_x_cm']:.1f},{event['target_y_cm']:.1f},"
                    f"{event['target_z_cm']:.1f},{event['suc_dir_deg']:.1f}) "
                    f"joint=({event['th0_deg']:.1f},{event['th1_deg']:.1f},"
                    f"{event['th2_deg']:.1f},{event['th3_deg']:.1f})",
                    flush=True,
                )
            if self.on_arm_event:
                self.on_arm_event(event)
        else:
            with self._state_lock:
                self.state["counters"]["unknown"] += 1
            if self.print_received:
                print(f"{self.rx_print_prefix}FRAME cmd=0x{cmd:02X} payload={payload.hex(' ')}", flush=True)

    def _handle_raw_bytes(self, data):
        if not data:
            return
        encoding = config_get(self.config, "serial.encoding", "utf-8")
        text = bytes(data).decode(encoding, errors="ignore").strip()
        if not text:
            return
        self._store_raw_text(text)
        if self.print_received:
            print(f"{self.rx_print_prefix}[RAW] {text}", flush=True)
        if self.on_raw_text:
            self.on_raw_text(text)
        elif self.on_text:
            self.on_text(text)

    def _receive_loop(self):
        buffer = bytearray()
        sleep_s = config_get(self.config, "serial.read_sleep_s", 0.01)
        header = bytes(config_get(self.config, "protocol.header", [0xAA, 0x55]))
        tail = bytes(config_get(self.config, "protocol.tail", [0x0D, 0x0A]))
        max_raw_len = int(config_get(self.config, "serial.max_raw_buffer_len", 256))

        while self._receiving:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    buffer.extend(self.serial_port.read(self.serial_port.in_waiting))

                    while True:
                        start = buffer.find(header)
                        if start < 0:
                            if b"\n" in buffer or b"\r" in buffer or len(buffer) >= max_raw_len:
                                self._handle_raw_bytes(buffer)
                                buffer.clear()
                            break
                        if start > 0:
                            self._handle_raw_bytes(buffer[:start])
                            del buffer[:start]
                        if len(buffer) < 9:
                            break
                        payload_len = buffer[5]
                        frame_len = 9 + payload_len
                        if len(buffer) < frame_len:
                            break
                        frame = bytes(buffer[:frame_len])
                        del buffer[:frame_len]
                        if frame[-2:] != tail:
                            continue
                        parsed = self.parse_frame(frame)
                        self._handle_frame(parsed)
            except Exception as exc:
                if self.on_error:
                    self.on_error(exc)
                elif self.print_received:
                    print(f"{self.rx_print_prefix}RX error: {exc}", flush=True)
            time.sleep(sleep_s)
