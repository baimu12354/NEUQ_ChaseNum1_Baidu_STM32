from __future__ import annotations

import argparse
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

from serial.tools import list_ports

UPPER_CONTROLLER_DIR = Path(__file__).resolve().parents[1] / "UpperController"
sys.path.insert(0, str(UPPER_CONTROLLER_DIR))

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


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


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


def has_ack_or_status(controller: ChassisController, frames: list[ChassisFrame]) -> bool:
    return any(frame.cmd in (controller.responses["ack"], controller.responses["status"]) for frame in frames)


class ChassisMonitorApp:
    def __init__(self, root: tk.Tk, config_path: Path, initial_port: str | None, timeout_s: float) -> None:
        self.root = root
        self.config_path = config_path
        self.controller: ChassisController | None = None
        self.reader_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.serial_lock = threading.Lock()
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.rx_count = 0

        self.root.title("Chassis Serial Monitor")
        self.root.geometry("1080x720")
        self.root.minsize(900, 600)

        self.port_var = tk.StringVar(value=initial_port or "")
        self.timeout_var = tk.DoubleVar(value=max(0.1, timeout_s))
        self.feedback_var = tk.StringVar(value="0")
        self.yaw_var = tk.StringVar(value="0.0")
        self.vx_var = tk.StringVar(value="0.0")
        self.vy_var = tk.StringVar(value="0.0")
        self.wz_var = tk.StringVar(value="0.0")
        self.autoscroll_var = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(value="Disconnected")

        self._build_ui()
        self.refresh_ports(select_initial=True)
        self._set_command_state("disabled")
        self.root.after(50, self._drain_events)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        conn = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 6))
        conn.columnconfigure(1, weight=1)

        ttk.Label(conn, text="Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="w", padx=(8, 16))

        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 8))
        self.connect_btn = ttk.Button(conn, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=(0, 16))

        ttk.Label(conn, text="Command timeout(s)").grid(row=0, column=4, sticky="e")
        ttk.Spinbox(conn, from_=0.1, to=5.0, increment=0.1, textvariable=self.timeout_var, width=8).grid(
            row=0, column=5, sticky="w", padx=(8, 16)
        )

        ttk.Label(conn, textvariable=self.status_var).grid(row=0, column=6, sticky="e")

        commands = ttk.LabelFrame(self.root, text="Commands", padding=10)
        commands.grid(row=1, column=0, sticky="ew", padx=10, pady=6)
        for col in range(10):
            commands.columnconfigure(col, weight=0)
        commands.columnconfigure(9, weight=1)

        self.command_widgets: list[tk.Widget] = []

        self._add_button(commands, "HELLO", lambda: self.send_command("hello"), 0, 0)
        self._add_button(commands, "GET_STATUS", lambda: self.send_command("get_status"), 0, 1)
        self._add_button(commands, "BRAKE", lambda: self.send_command("brake"), 0, 2)
        self._add_button(commands, "RUN 0", lambda: self.send_run(0.0, 0.0, 0.0), 0, 3)

        ttk.Label(commands, text="Feedback ms").grid(row=1, column=0, sticky="w", pady=(10, 0))
        feedback_entry = ttk.Entry(commands, textvariable=self.feedback_var, width=10)
        feedback_entry.grid(row=1, column=1, sticky="w", padx=(8, 8), pady=(10, 0))
        self.command_widgets.append(feedback_entry)
        self._add_button(commands, "SET_FEEDBACK", self.send_feedback, 1, 2, pady=(10, 0))

        ttk.Label(commands, text="Yaw").grid(row=1, column=3, sticky="e", padx=(20, 0), pady=(10, 0))
        yaw_entry = ttk.Entry(commands, textvariable=self.yaw_var, width=10)
        yaw_entry.grid(row=1, column=4, sticky="w", padx=(8, 8), pady=(10, 0))
        self.command_widgets.append(yaw_entry)
        self._add_button(commands, "SET_YAW", self.send_yaw, 1, 5, pady=(10, 0))

        ttk.Label(commands, text="vx").grid(row=2, column=0, sticky="w", pady=(10, 0))
        vx_entry = ttk.Entry(commands, textvariable=self.vx_var, width=10)
        vx_entry.grid(row=2, column=1, sticky="w", padx=(8, 8), pady=(10, 0))
        self.command_widgets.append(vx_entry)

        ttk.Label(commands, text="vy").grid(row=2, column=2, sticky="e", pady=(10, 0))
        vy_entry = ttk.Entry(commands, textvariable=self.vy_var, width=10)
        vy_entry.grid(row=2, column=3, sticky="w", padx=(8, 8), pady=(10, 0))
        self.command_widgets.append(vy_entry)

        ttk.Label(commands, text="wz").grid(row=2, column=4, sticky="e", pady=(10, 0))
        wz_entry = ttk.Entry(commands, textvariable=self.wz_var, width=10)
        wz_entry.grid(row=2, column=5, sticky="w", padx=(8, 8), pady=(10, 0))
        self.command_widgets.append(wz_entry)
        self._add_button(commands, "RUN", self.send_run_from_inputs, 2, 6, pady=(10, 0))

        log_frame = ttk.Frame(self.root, padding=(10, 6, 10, 10))
        log_frame.grid(row=2, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(log_frame, wrap="none", height=22, font=("Consolas", 10), state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        y_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        y_scroll.grid(row=0, column=1, sticky="ns")
        x_scroll = ttk.Scrollbar(log_frame, orient="horizontal", command=self.log_text.xview)
        x_scroll.grid(row=1, column=0, sticky="ew")
        self.log_text.configure(yscrollcommand=y_scroll.set, xscrollcommand=x_scroll.set)

        self.log_text.tag_configure("tx", foreground="#0A5")
        self.log_text.tag_configure("rx", foreground="#06C")
        self.log_text.tag_configure("error", foreground="#B00020")
        self.log_text.tag_configure("info", foreground="#555")

        footer = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        footer.grid(row=3, column=0, sticky="ew")
        footer.columnconfigure(0, weight=1)
        ttk.Checkbutton(footer, text="Auto scroll", variable=self.autoscroll_var).grid(row=0, column=0, sticky="w")
        ttk.Button(footer, text="Clear Log", command=self.clear_log).grid(row=0, column=1, sticky="e")

    def _add_button(
        self,
        parent: tk.Widget,
        text: str,
        command: object,
        row: int,
        column: int,
        pady: tuple[int, int] | int = 0,
    ) -> ttk.Button:
        button = ttk.Button(parent, text=text, command=command)
        button.grid(row=row, column=column, sticky="w", padx=(0, 8), pady=pady)
        self.command_widgets.append(button)
        return button

    def refresh_ports(self, select_initial: bool = False) -> None:
        ports = available_ports()
        self.port_combo["values"] = ports
        if select_initial and self.port_var.get() in ports:
            return
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")

    def toggle_connection(self) -> None:
        if self.controller is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Port required", "No serial port selected.")
            return

        try:
            controller = ChassisController(self.config_path)
            controller.config["serial"]["port"] = port
            controller.open()
            if controller.serial is not None:
                controller.serial.reset_input_buffer()
                controller.serial.reset_output_buffer()
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))
            self.log(f"ERROR: {exc}", "error")
            return

        self.controller = controller
        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.connect_btn.configure(text="Disconnect")
        self.port_combo.configure(state="disabled")
        self._set_command_state("normal")
        self.status_var.set(f"Connected: {port}")
        self.log(f"Connected to {port} at {controller.config['serial']['baudrate']} baud", "info")

    def disconnect(self) -> None:
        self.stop_event.set()
        controller = self.controller
        self.controller = None
        if controller is not None:
            try:
                controller.close()
            except Exception as exc:
                self.log(f"ERROR while closing port: {exc}", "error")
        if self.reader_thread is not None and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.3)
        self.reader_thread = None
        self.connect_btn.configure(text="Connect")
        self.port_combo.configure(state="readonly")
        self._set_command_state("disabled")
        self.status_var.set("Disconnected")
        self.log("Disconnected", "info")

    def on_close(self) -> None:
        self.disconnect()
        self.root.destroy()

    def _set_command_state(self, state: str) -> None:
        for widget in self.command_widgets:
            try:
                widget.configure(state=state)
            except tk.TclError:
                pass

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            controller = self.controller
            if controller is None:
                break
            try:
                with self.serial_lock:
                    frame = controller.read_raw_frame()
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.events.put(("error", f"RX ERROR: {exc}"))
                break
            if frame is not None:
                self.events.put(("rx", frame))

    def _drain_events(self) -> None:
        while True:
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "rx" and isinstance(value, ChassisFrame):
                self.rx_count += 1
                controller = self.controller
                if controller is not None:
                    self.log(f"RX {describe_frame(controller, value)}", "rx")
            elif kind == "error":
                self.log(str(value), "error")
        self.root.after(50, self._drain_events)

    def send_command(self, name: str, payload: bytes = b"") -> None:
        controller = self.controller
        if controller is None:
            messagebox.showwarning("Not connected", "Connect to a serial port first.")
            return
        try:
            before = self.rx_count
            with self.serial_lock:
                tx = controller.send_command(name, payload)
            self.log(f"TX {name}: {controller.frame_to_hex(tx)}", "tx")
            self.root.after(self.command_timeout_ms(), lambda: self._check_timeout(name, before))
        except Exception as exc:
            self.log(f"TX ERROR {name}: {exc}", "error")

    def send_feedback(self) -> None:
        try:
            interval_ms = max(0, min(65535, int(self.feedback_var.get().strip())))
        except ValueError:
            messagebox.showerror("Bad value", "Feedback interval must be an integer.")
            return
        self.send_command("set_feedback", struct.pack("<H", interval_ms))

    def send_yaw(self) -> None:
        try:
            yaw = float(self.yaw_var.get().strip())
        except ValueError:
            messagebox.showerror("Bad value", "Yaw must be a number.")
            return
        self.send_command("set_yaw", struct.pack("<f", yaw))

    def send_run_from_inputs(self) -> None:
        try:
            vx = float(self.vx_var.get().strip())
            vy = float(self.vy_var.get().strip())
            wz = float(self.wz_var.get().strip())
        except ValueError:
            messagebox.showerror("Bad value", "vx, vy, and wz must be numbers.")
            return
        self.send_run(vx, vy, wz)

    def send_run(self, vx: float, vy: float, wz: float) -> None:
        self.send_command("run", struct.pack("<fff", vx, vy, wz))

    def command_timeout_ms(self) -> int:
        try:
            return max(100, int(float(self.timeout_var.get()) * 1000))
        except (tk.TclError, ValueError):
            return 500

    def _check_timeout(self, name: str, rx_before: int) -> None:
        if self.controller is not None and self.rx_count == rx_before:
            self.log(f"RX {name}: <timeout>", "error")

    def log(self, message: str, tag: str = "info") -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {message}\n", tag)
        self.log_text.configure(state="disabled")
        if self.autoscroll_var.get():
            self.log_text.see("end")

    def clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")


def run_cli(args: argparse.Namespace) -> int:
    controller = ChassisController(args.config)
    if args.port:
        controller.config["serial"]["port"] = args.port

    port = controller.config["serial"]["port"]
    ports = available_ports()
    if port not in ports:
        print(f"ERROR: serial port {port!r} was not found.", file=sys.stderr)
        if ports:
            print(f"Available ports: {', '.join(ports)}", file=sys.stderr)
            print(f"Try: python {Path(__file__).name} --port {ports[0]}", file=sys.stderr)
        else:
            print("No serial ports were detected. Check USB cable, driver, and board power.", file=sys.stderr)
        return 2

    print(f"Opening {port} at {controller.config['serial']['baudrate']} baud")

    try:
        with controller:
            if controller.serial is not None:
                controller.serial.reset_input_buffer()
            ok = True

            frames = send_and_print(controller, "hello", timeout_s=args.timeout)
            ok = ok and has_ack_or_status(controller, frames)

            frames = send_and_print(controller, "get_status", timeout_s=args.timeout)
            ok = ok and has_status(controller, frames)

            payload = struct.pack("<H", max(0, min(65535, args.feedback_ms)))
            frames = send_and_print(controller, "set_feedback", payload, timeout_s=args.timeout)
            ok = ok and has_ack_or_status(controller, frames)

            if args.set_yaw is not None:
                frames = send_and_print(controller, "set_yaw", struct.pack("<f", args.set_yaw), timeout_s=args.timeout)
                ok = ok and has_status(controller, frames)

            if args.unsafe_run_test:
                frames = send_and_print(controller, "run", struct.pack("<fff", 0.0, 0.0, 0.0), timeout_s=args.timeout)
                ok = ok and has_status(controller, frames)

            frames = send_and_print(controller, "brake", timeout_s=args.timeout)
            ok = ok and has_ack_or_status(controller, frames)

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


def run_gui(args: argparse.Namespace) -> int:
    root = tk.Tk()
    ChassisMonitorApp(root, args.config, args.port, args.timeout)
    root.mainloop()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Visual test tool for the upper/lower controller serial protocol.")
    parser.add_argument("--config", type=Path, default=UPPER_CONTROLLER_DIR / "communication.yml")
    parser.add_argument("--port", help="Override serial port, for example COM5 or /dev/ttyUSB0.")
    parser.add_argument("--timeout", type=float, default=0.5, help="Receive window per test step in seconds.")
    parser.add_argument("--set-yaw", type=float, default=None, help="CLI only: optional yaw value to test SET_YAW.")
    parser.add_argument("--feedback-ms", type=int, default=0, help="CLI only: feedback interval to test.")
    parser.add_argument("--unsafe-run-test", action="store_true", help="CLI only: also send a zero-speed RUN command.")
    parser.add_argument("--cli", action="store_true", help="Run the original command-line test instead of the GUI.")
    args = parser.parse_args()

    if args.cli:
        return run_cli(args)
    return run_gui(args)


if __name__ == "__main__":
    raise SystemExit(main())
