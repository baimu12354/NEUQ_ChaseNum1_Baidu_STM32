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
        "header": [0x5A, 0xA5],
        "tail": [0x0D, 0x0A],
        "cmd_stop": 0x00,
        "cmd_move": 0x01,
        "cmd_fold": 0x02,
        "move_payload_format": "<ffff",
    },
    "ui_defaults": {
        "world_x_cm": 0.0,
        "world_y_cm": 30.0,
        "world_z_cm": 18.0,
        "suc_dir_deg": 0.0,
    },
    "arm_geometry": {
        "h0_cm": 8.8,
        "h1_cm": 6.5,
        "r_cm": 3.0,
        "l1_cm": 17.5,
        "c_cm": 2.5,
        "l2_cm": 20.5,
        "suction_len_cm": 11.0,
    },
    "front_rod": {
        "enabled": True,
        "x_cm": 0.0,
        "y_cm": 30.0,
        "radius_cm": 1.5,
    },
    "planner_preview": {
        "x_side_eps_cm": 1.0,
    },
    "folded_pose": {
        "default_base_deg": 90.0,
        "th1_deg": 120.0,
        "th2_deg": -145.0,
        "th3_deg": 75.0,
    },
    "servo_mapping": {
        "s0_offset_deg": 5.0,
        "s1_gear": 3.0,
        "s1_offset_deg": -533.0,
        "s2_offset_deg": -13.0,
        "s3_offset_deg": -97.0,
    },
    "servo_limits": {
        "s0_min_deg": -180.0,
        "s0_max_deg": 180.0,
        "s1_min_deg": -533.0,
        "s1_max_deg": 7.0,
        "s2_min_deg": -90.0,
        "s2_max_deg": 150.0,
        "s3_min_deg": -100.0,
        "s3_max_deg": 180.0,
    },
}


def config_get(config, dotted_key, default=None):
    current = config
    for key in dotted_key.split("."):
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


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
    if lowered in ("none", "null"):
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


def load_arm_communication_config(config_path=None):
    if config_path is None:
        config_path = Path(__file__).with_name("ARM_communication.yml")
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


class ArmCommunicationController:
    def __init__(
            self,
            config=None,
            config_path=None,
            on_message=None,
            on_error=None,
            print_received=None,
            rx_print_prefix=None
    ):
        self.config = config if config is not None else load_arm_communication_config(config_path)
        self.on_message = on_message
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
        self._receiving = False
        self._receiver_thread = None

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
        if start_receiver:
            self.start_receiver()
        return self.serial_port

    def disconnect(self):
        self.stop_receiver()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def close(self):
        self.disconnect()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def set_message_callback(self, callback):
        self.on_message = callback

    def set_error_callback(self, callback):
        self.on_error = callback

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

    def build_packet(self, cmd_type, payload=b""):
        header = bytes(config_get(self.config, "protocol.header", [0x5A, 0xA5]))
        tail = bytes(config_get(self.config, "protocol.tail", [0x0D, 0x0A]))
        checksum = (cmd_type + sum(payload)) & 0xFF
        return header + struct.pack("B", cmd_type) + payload + struct.pack("B", checksum) + tail

    def build_target_payload(self, world_x, world_y, world_z, suc_dir_deg):
        payload_format = config_get(self.config, "protocol.move_payload_format", "<ffff")
        return struct.pack(payload_format, float(world_x), float(world_y), float(world_z), float(suc_dir_deg))

    def send_packet(self, cmd_type, payload=b""):
        if not self.is_open:
            raise RuntimeError("串口未连接")
        packet = self.build_packet(cmd_type, payload)
        self.serial_port.write(packet)
        return packet

    def send_target(self, world_x, world_y, world_z, suc_dir_deg):
        cmd_move = config_get(self.config, "protocol.cmd_move", 0x01)
        payload = self.build_target_payload(world_x, world_y, world_z, suc_dir_deg)
        return self.send_packet(cmd_move, payload)

    def send_stop(self):
        cmd_stop = config_get(self.config, "protocol.cmd_stop", 0x00)
        return self.send_packet(cmd_stop)

    def send_fold(self):
        cmd_fold = config_get(self.config, "protocol.cmd_fold", 0x02)
        return self.send_packet(cmd_fold)

    def _receive_loop(self):
        buffer = ""
        encoding = config_get(self.config, "serial.encoding", "utf-8")
        sleep_s = config_get(self.config, "serial.read_sleep_s", 0.01)

        while self._receiving:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer += data.decode(encoding, errors="ignore")

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            if self.print_received:
                                print(f"{self.rx_print_prefix}{line}", flush=True)
                            if self.on_message:
                                self.on_message(line)
            except Exception as exc:
                if self.on_error:
                    self.on_error(exc)
            time.sleep(sleep_s)
