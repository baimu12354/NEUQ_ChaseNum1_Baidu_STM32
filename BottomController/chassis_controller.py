from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import serial
import yaml


@dataclass
class ChassisStatus:
    """下位机 STATUS 响应解析后的结构化状态。"""

    mode: int
    yaw_deg: float
    vx: float
    vy: float
    wz_deg_s: float
    timestamp_ms: int


@dataclass
class ChassisFrame:
    """一帧完整的下位机响应，保留原始字节用于调试抓包。"""

    cmd: int
    payload: bytes
    seq: int
    raw: bytes


@dataclass
class ChassisAck:
    """ACK/ERR 响应的载荷结构：原始命令字 + 错误码。"""

    original_cmd: int
    error: int


class ChassisController:
    """上位机底盘通信控制器。

    公开方法用于业务调用；以下划线开头的方法用于协议组包、拆包和解析。
    """

    def __init__(self, config_path: str | Path | None = None) -> None:
        # 默认读取本文件同目录下的 communication.yml，方便直接运行脚本。
        self.config_path = Path(config_path) if config_path is not None else Path(__file__).with_name("communication.yml")
        self.config = self._load_config(self.config_path)
        self.protocol = self.config["protocol"]
        self.commands = self.config["commands"]
        self.responses = self.config["responses"]
        self.limits = self.config["limits"]
        # 上位机发送帧的序号，按 0-255 循环递增；下位机不会依赖该值执行业务。
        self.seq = 0
        self.serial: serial.Serial | None = None
        # 接收缓冲区用于处理串口分包、粘包和前导噪声。
        self._rx_buffer = bytearray()

    def open(self) -> None:
        """按 YAML 配置打开串口。"""

        cfg = self.config["serial"]
        self.serial = serial.Serial(
            port=cfg["port"],
            baudrate=int(cfg["baudrate"]),
            bytesize=int(cfg["bytesize"]),
            parity=str(cfg["parity"]),
            stopbits=int(cfg["stopbits"]),
            timeout=float(cfg["timeout_s"]),
            write_timeout=float(cfg["write_timeout_s"]),
        )

    def close(self) -> None:
        """关闭串口连接。"""

        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def __enter__(self) -> "ChassisController":
        """支持 with ChassisController() as chassis 的写法。"""

        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        """退出 with 块时自动关闭串口。"""

        self.close()

    def hello(self, timeout_s: float = 0.5) -> ChassisStatus | None:
        """通信握手：发送 HELLO，并等待下位机 STATUS。"""

        self._send_command("hello")
        return self._wait_status(timeout_s)

    def brake(self, timeout_s: float = 0.5) -> ChassisStatus | None:
        """刹车：请求下位机进入 Movement_Breaking。"""

        self._send_command("brake")
        return self._wait_status(timeout_s)

    def run(self, vx: float, vy: float, wz_deg_s: float, timeout_s: float = 0.2) -> ChassisStatus | None:
        """发送一次速度控制命令。

        注意：RUN 是速度命令，不是持续任务；连续运动需要周期调用，或使用 run_for()。
        """

        # 上位机先做一次限幅；下位机也会二次限幅，双保险。
        vx = self._limit(vx, self.limits["max_vx"])
        vy = self._limit(vy, self.limits["max_vy"])
        wz_deg_s = self._limit(wz_deg_s, self.limits["max_wz_deg_s"])
        # payload 格式：float32 vx + float32 vy + float32 wz，均为 little-endian。
        self._send_command("run", struct.pack("<fff", vx, vy, wz_deg_s))
        return self._wait_status(timeout_s)

    def run_for(self, vx: float, vy: float, wz_deg_s: float, duration_s: float, rate_hz: float = 20.0) -> None:
        """按固定频率重复发送 RUN，持续指定时间，结束后自动刹车。"""

        period_s = 1.0 / max(1.0, float(rate_hz))
        deadline = time.monotonic() + max(0.0, float(duration_s))
        while time.monotonic() < deadline:
            self.run(vx, vy, wz_deg_s, timeout_s=0.05)
            time.sleep(period_s)
        self.brake(timeout_s=0.2)

    def drive_running_mode(self, ch1: float, ch2: float, timeout_s: float = 0.2) -> ChassisStatus | None:
        """按 RC_control() 中的“行驶/转向”模式模拟遥控器输入。"""

        mapping = self.config["rc_mapping"]
        center = float(mapping["channel_center"])
        mode = mapping["running"]
        # 对应下位机 RC_control(): vy=(ch2-center)/20, wz=(ch1-center)/5, vx=0。
        vy = (ch2 - center) / float(mode["vy_scale"])
        wz = (ch1 - center) / float(mode["wz_scale"])
        return self.run(float(mode["vx"]), vy, wz, timeout_s)

    def drive_moving_mode(self, ch1: float, ch2: float, timeout_s: float = 0.2) -> ChassisStatus | None:
        """按 RC_control() 中的“平移”模式模拟遥控器输入。"""

        mapping = self.config["rc_mapping"]
        center = float(mapping["channel_center"])
        mode = mapping["moving"]
        # 对应下位机 RC_control(): vx=(ch1-center)/50, vy=(ch2-center)/20, wz=0。
        vx = (ch1 - center) / float(mode["vx_scale"])
        vy = (ch2 - center) / float(mode["vy_scale"])
        return self.run(vx, vy, float(mode["wz"]), timeout_s)

    def get_status(self, timeout_s: float = 0.5) -> ChassisStatus | None:
        """主动请求一次 STATUS，读取航向角和当前速度目标。"""

        self._send_command("get_status")
        return self._wait_status(timeout_s)

    def set_yaw(self, yaw_deg: float, timeout_s: float = 0.5) -> ChassisStatus | None:
        """设置下位机逻辑航向角。"""

        self._send_command("set_yaw", struct.pack("<f", float(yaw_deg)))
        return self._wait_status(timeout_s)

    def set_feedback_interval(self, interval_ms: int, timeout_s: float = 0.5) -> ChassisStatus | None:
        """设置下位机周期 STATUS 反馈间隔；0 表示关闭周期反馈。"""

        interval_ms = max(0, min(65535, int(interval_ms)))
        self._send_command("set_feedback", struct.pack("<H", interval_ms))
        return self._wait_status(timeout_s)

    def read_frame(self) -> tuple[int, bytes] | None:
        """读取一帧响应，只返回 cmd 和 payload，兼容简单调用。"""

        frame = self.read_raw_frame()
        if frame is None:
            return None
        return frame.cmd, frame.payload

    def read_raw_frame(self) -> ChassisFrame | None:
        """读取一帧响应，返回包含原始字节的 ChassisFrame。"""

        # 先尝试从已有缓冲区解析，避免串口 timeout 带来的多余等待。
        frame = self._pop_frame()
        if frame is not None:
            return frame

        ser = self._require_serial()
        # 如果驱动缓冲区已有数据，就一次性读出；否则阻塞读 1 字节等待新数据。
        data = ser.read(ser.in_waiting or 1)
        if data:
            self._rx_buffer.extend(data)
        return self._pop_frame()

    def send_command(self, name: str, payload: bytes = b"") -> bytes:
        """发送指定命令，返回本次实际写入串口的原始帧，便于打印调试。"""

        return self._send_command(name, payload)

    def request_raw(self, name: str, payload: bytes = b"", timeout_s: float = 0.5) -> list[ChassisFrame]:
        """发送命令并在指定时间内收集所有下位机响应帧。"""

        self._send_command(name, payload)
        return self.collect_frames(timeout_s)

    def collect_frames(self, timeout_s: float = 0.5) -> list[ChassisFrame]:
        """在 timeout_s 时间窗口内尽可能收集下位机返回的所有完整帧。"""

        frames: list[ChassisFrame] = []
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self.read_raw_frame()
            if frame is None:
                continue
            frames.append(frame)
        return frames

    def listen_status(self, callback: Callable[[ChassisStatus], None], duration_s: float | None = None) -> None:
        """持续监听 STATUS，并把解析后的 ChassisStatus 交给 callback。"""

        start = time.monotonic()
        while duration_s is None or time.monotonic() - start < duration_s:
            frame = self.read_frame()
            if frame is None:
                continue
            cmd, payload = frame
            if cmd == self.responses["status"]:
                callback(self._parse_status(payload))

    def parse_frame_payload(self, frame: ChassisFrame) -> ChassisStatus | ChassisAck | bytes:
        """根据响应类型解析 payload。

        STATUS -> ChassisStatus；ACK/ERR -> ChassisAck；未知响应 -> 原始 bytes。
        """

        if frame.cmd == self.responses["status"]:
            return self._parse_status(frame.payload)
        if frame.cmd in (self.responses["ack"], self.responses["error"]) and len(frame.payload) >= 2:
            return ChassisAck(frame.payload[0], frame.payload[1])
        return frame.payload

    @staticmethod
    def frame_to_hex(frame: bytes | ChassisFrame) -> str:
        """把原始帧格式化为便于粘贴到文档或串口助手中的十六进制字符串。"""

        raw = frame.raw if isinstance(frame, ChassisFrame) else frame
        return " ".join(f"{byte:02X}" for byte in raw)

    def _send_command(self, name: str, payload: bytes = b"") -> bytes:
        """按命令名称查表并发送协议帧。"""

        return self._write_frame(self.commands[name], payload)

    def _write_frame(self, cmd: int, payload: bytes) -> bytes:
        """底层组包发送函数。"""

        ser = self._require_serial()
        if len(payload) > int(self.protocol["max_payload_len"]):
            raise ValueError("payload too long")

        version = int(self.protocol["version"])
        seq = self.seq & 0xFF
        self.seq = (self.seq + 1) & 0xFF
        length = len(payload)
        # 校验和必须与下位机 Main_COM_calc_check() 保持一致。
        check = (version + seq + cmd + length + sum(payload)) & 0xFF
        frame = bytes(self.protocol["header"]) + bytes([version, seq, cmd, length]) + payload + bytes([check]) + bytes(
            self.protocol["tail"]
        )
        ser.write(frame)
        return frame

    def _wait_status(self, timeout_s: float) -> ChassisStatus | None:
        """等待 STATUS 响应。

        高级接口通常会收到 ACK + STATUS；这里会忽略 ACK，遇到 ERR 时记录并在超时后抛出异常。
        """

        deadline = time.monotonic() + timeout_s
        last_ack_error: tuple[int, int] | None = None
        while time.monotonic() < deadline:
            frame = self.read_raw_frame()
            if frame is None:
                continue
            if frame.cmd == self.responses["status"]:
                return self._parse_status(frame.payload)
            if frame.cmd == self.responses["error"] and len(frame.payload) >= 2:
                last_ack_error = (frame.payload[0], frame.payload[1])
        if last_ack_error is not None:
            raise RuntimeError(f"lower controller error: cmd=0x{last_ack_error[0]:02X}, err=0x{last_ack_error[1]:02X}")
        return None

    def _pop_frame(self) -> ChassisFrame | None:
        """从接收缓冲区中解析一帧完整协议帧。

        该函数处理三类串口常见情况：前导噪声、半包、粘包。
        """

        header = bytes(self.protocol["header"])
        tail = bytes(self.protocol["tail"])
        version = int(self.protocol["version"])
        max_payload = int(self.protocol["max_payload_len"])

        while True:
            # 丢弃帧头前的噪声字节。
            head_index = self._rx_buffer.find(header)
            if head_index < 0:
                self._rx_buffer.clear()
                return None
            if head_index > 0:
                del self._rx_buffer[:head_index]
            if len(self._rx_buffer) < 9:
                # 最短帧长度不足，等待更多串口数据。
                return None

            payload_len = self._rx_buffer[5]
            if payload_len > max_payload:
                # 长度明显非法，丢弃当前 AA，继续找下一帧头。
                del self._rx_buffer[0]
                continue

            frame_len = 9 + payload_len
            if len(self._rx_buffer) < frame_len:
                # 半包，等待后续字节。
                return None

            frame = bytes(self._rx_buffer[:frame_len])
            del self._rx_buffer[:frame_len]

            if frame[2] != version or frame[-2:] != tail:
                continue
            payload = frame[6 : 6 + payload_len]
            check = sum(frame[2:6]) + sum(payload)
            if (check & 0xFF) != frame[6 + payload_len]:
                # 校验失败，丢弃该帧；上层可通过超时重发。
                continue
            return ChassisFrame(cmd=frame[4], payload=payload, seq=frame[3], raw=frame)

    @staticmethod
    def _parse_status(payload: bytes) -> ChassisStatus:
        """解析 STATUS payload。

        payload 格式：mode:uint8 + yaw/vx/vy/wz:float32 * 4 + timestamp:uint32。
        """

        if len(payload) != 21:
            raise ValueError(f"bad status payload length: {len(payload)}")
        mode = payload[0]
        yaw, vx, vy, wz = struct.unpack("<ffff", payload[1:17])
        timestamp_ms = struct.unpack("<I", payload[17:21])[0]
        return ChassisStatus(mode, yaw, vx, vy, wz, timestamp_ms)

    @staticmethod
    def _limit(value: float, max_abs: float) -> float:
        """把 value 限制到 [-max_abs, +max_abs]。"""

        value = float(value)
        max_abs = float(max_abs)
        return max(-max_abs, min(max_abs, value))

    @staticmethod
    def _load_config(path: Path) -> dict[str, Any]:
        """读取 YAML 配置文件。"""

        with path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def _require_serial(self) -> serial.Serial:
        """获取已打开的串口对象；未打开时给出明确错误。"""

        if self.serial is None:
            raise RuntimeError("serial port is not open")
        return self.serial
