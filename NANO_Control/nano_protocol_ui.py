import queue
import math
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


HEAD = b"\xAA\x55"
TAIL = b"\x0D\x0A"
VERSION = 0x01

MODULE_SYSTEM = 0x01
MODULE_CHASSIS = 0x02
MODULE_ARM = 0x03
MODULE_PUMP = 0x04
MODULE_SHOOTER = 0x05

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
CMD_CHASSIS_START_LOCAL_POS = 0x16
CMD_CHASSIS_GET_LOCAL_POS = 0x17
CMD_CHASSIS_STOP_LOCAL_POS = 0x18

CMD_ARM_TARGET_XYZ_DIR = 0x20
CMD_ARM_FOLD_HOME = 0x21
CMD_ARM_STOP_HOLD = 0x22
CMD_ARM_GET_POSE = 0x23

CMD_PUMP_SUCK = 0x30
CMD_PUMP_RELEASE = 0x31
CMD_PUMP_GET_STATE = 0x32

CMD_SHOOTER_FIRE = 0x40
CMD_SHOOTER_RESET = 0x41
CMD_SHOOTER_GET_STATE = 0x42

RESP_ACK = 0x80
RESP_NACK = 0x81
RESP_STATUS = 0x82
RESP_EVENT = 0x83
RESP_ARM_POSE = 0x84
RESP_CHASSIS_STATE = 0x85
RESP_PUMP_STATE = 0x86
RESP_CHASSIS_LOCAL_POS = 0x87
RESP_SHOOTER_STATE = 0x88

MODULE_NAMES = {
    MODULE_SYSTEM: "SYSTEM",
    MODULE_CHASSIS: "CHASSIS",
    MODULE_ARM: "ARM",
    MODULE_PUMP: "PUMP",
    MODULE_SHOOTER: "SHOOTER",
}

RESP_NAMES = {
    RESP_ACK: "ACK",
    RESP_NACK: "NACK",
    RESP_STATUS: "STATUS",
    RESP_EVENT: "EVENT",
    RESP_ARM_POSE: "ARM_POSE",
    RESP_CHASSIS_STATE: "CHASSIS_STATE",
    RESP_PUMP_STATE: "PUMP_STATE",
    RESP_CHASSIS_LOCAL_POS: "CHASSIS_LOCAL_POS",
    RESP_SHOOTER_STATE: "SHOOTER_STATE",
}

SYS_STATES = ["INIT", "RET", "IDLE", "ACTIVE", "ESTOP", "FAULT"]
CHASSIS_STATES = ["IDLE", "RUN", "ROTATE", "TRANSLATE", "STOP", "BRAKE", "FAULT"]
ARM_STATES = ["IDLE", "MOVING", "FOLDING", "HOLD", "REJECTED", "FAULT"]
PUMP_STATES = ["IDLE", "SUCKING", "SUCKED", "RELEASING", "RELEASED", "FAULT"]
SHOOTER_STATES = ["IDLE", "SPINUP", "PUSHING", "RETURNING", "RESETTING", "FAULT"]

EVENT_NAMES = {
    0x01: "SYS_READY",
    0x02: "SYS_RESET_DONE",
    0x03: "SYS_ESTOP",
    0x10: "CHASSIS_ROTATE_DONE",
    0x11: "CHASSIS_TRANSLATE_DONE",
    0x12: "CHASSIS_TIMEOUT_STOP",
    0x20: "ARM_DONE",
    0x21: "ARM_FOLD_DONE",
    0x22: "ARM_REJECTED",
    0x30: "PUMP_SUCK_DONE",
    0x31: "PUMP_RELEASE_DONE",
    0x32: "PUMP_TIMEOUT_STOP",
    0x40: "SHOOTER_DONE",
    0x41: "SHOOTER_RESET_DONE",
    0x42: "SHOOTER_TIMEOUT",
}

ERR_NAMES = {
    0x00: "OK",
    0x01: "BAD_CMD",
    0x02: "BAD_LEN",
    0x03: "BAD_CHECK",
    0x04: "BAD_STATE",
    0x05: "BUSY",
    0x06: "RANGE_LIMIT",
    0x07: "FORBIDDEN_ZONE",
    0x08: "TIMEOUT",
    0x09: "DEVICE",
}

ARM_H0 = 11.8
ARM_H1 = 6.1
ARM_R = 3.1
ARM_L1 = 17.5
ARM_C = 2.0
ARM_L2 = 21.0
ARM_L3 = 9.0

RUN_SPEED_MAX = 100.0
RUN_WZ_MAX = 100.0
ROT_WZ_MIN = 1.0
ROT_WZ_MAX = 100.0
TRANS_DIST_MAX_CM = 1000.0
TRANS_SPEED_MIN = 1.0
TRANS_SPEED_MAX = 100.0


def state_name(table, value):
    if 0 <= value < len(table):
        return table[value]
    return f"UNKNOWN({value})"


def f32(*values):
    return struct.pack("<" + "f" * len(values), *values)


def read_f32(payload, offset):
    return struct.unpack_from("<f", payload, offset)[0]


class NanoCodec:
    def __init__(self):
        self.seq = 0
        self.rx_buf = bytearray()

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    @staticmethod
    def checksum(seq, module, cmd, payload):
        return (VERSION + seq + module + cmd + len(payload) + sum(payload)) & 0xFF

    def build(self, module, cmd, payload=b""):
        seq = self.next_seq()
        body = bytes([VERSION, seq, module, cmd, len(payload)]) + payload
        check = self.checksum(seq, module, cmd, payload)
        return seq, HEAD + body + bytes([check]) + TAIL

    def feed(self, data):
        frames = []
        self.rx_buf.extend(data)
        while True:
            head_index = self.rx_buf.find(HEAD)
            if head_index < 0:
                self.rx_buf.clear()
                break
            if head_index > 0:
                del self.rx_buf[:head_index]
            if len(self.rx_buf) < 9:
                break
            length = self.rx_buf[6]
            frame_len = 2 + 5 + length + 1 + 2
            if len(self.rx_buf) < frame_len:
                break
            frame = bytes(self.rx_buf[:frame_len])
            del self.rx_buf[:frame_len]
            if frame[-2:] != TAIL:
                continue
            version, seq, module, cmd, payload_len = frame[2], frame[3], frame[4], frame[5], frame[6]
            payload = frame[7:7 + payload_len]
            check = frame[7 + payload_len]
            if version == VERSION and check == self.checksum(seq, module, cmd, payload):
                frames.append((seq, module, cmd, payload, frame))
        return frames


class ArmView(tk.Canvas):
    def __init__(self, master):
        super().__init__(master, bg="#0b1220", highlightthickness=0)
        self.pose = None
        self.scale = 5.2
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_pose(self, pose):
        self.pose = pose
        self.redraw()

    def project(self, x, y, z):
        width = max(self.winfo_width(), 1)
        height = max(self.winfo_height(), 1)
        cx = width * 0.52
        cy = height * 0.78
        sx = cx + self.scale * (x - y) * 0.78
        sy = cy - self.scale * z + self.scale * (x + y) * 0.30
        return sx, sy

    def polyline3d(self, points, fill, width=2, dash=None):
        coords = []
        for point in points:
            coords.extend(self.project(*point))
        if len(coords) >= 4:
            self.create_line(*coords, fill=fill, width=width, dash=dash, capstyle=tk.ROUND, joinstyle=tk.ROUND)

    def draw_text3d(self, point, text, fill="#94a3b8"):
        sx, sy = self.project(*point)
        self.create_text(sx, sy, text=text, fill=fill, font=("Segoe UI", 9))

    def draw_box(self, xmin, xmax, ymin, ymax, zmin, zmax, color):
        corners = [
            (xmin, ymin, zmin), (xmax, ymin, zmin), (xmax, ymax, zmin), (xmin, ymax, zmin),
            (xmin, ymin, zmax), (xmax, ymin, zmax), (xmax, ymax, zmax), (xmin, ymax, zmax),
        ]
        edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
        for a, b in edges:
            self.polyline3d([corners[a], corners[b]], color, width=2, dash=(5, 4))

    def arm_points(self):
        if not self.pose:
            return [(0, 0, 0), (0, 0, ARM_H0)]

        _x, _y, _z, _direction, th0, th1, th2, th3 = self.pose
        r0 = th0 * 3.141592653589793 / 180.0
        r1 = th1 * 3.141592653589793 / 180.0
        r12 = (th1 + th2) * 3.141592653589793 / 180.0
        r123 = (th1 + th2 + th3) * 3.141592653589793 / 180.0

        def world(depth, z):
            return (depth * math.sin(r0), depth * math.cos(r0), z)

        base = (0.0, 0.0, 0.0)
        top = (0.0, 0.0, ARM_H0)
        shoulder = world(ARM_R, ARM_H0 + ARM_H1)
        joint2_depth = ARM_R + ARM_L1 * math.cos(r1) - ARM_C * math.sin(r1)
        joint2_z = ARM_H0 + ARM_H1 + ARM_L1 * math.sin(r1) + ARM_C * math.cos(r1)
        joint2 = world(joint2_depth, joint2_z)
        wrist_depth = joint2_depth + ARM_L2 * math.cos(r12)
        wrist_z = joint2_z + ARM_L2 * math.sin(r12)
        wrist = world(wrist_depth, wrist_z)
        end_depth = wrist_depth + ARM_L3 * math.cos(r123)
        end_z = wrist_z + ARM_L3 * math.sin(r123)
        end = world(end_depth, end_z)
        return [base, top, shoulder, joint2, wrist, end]

    def redraw(self):
        self.delete("all")
        self.create_rectangle(0, 0, self.winfo_width(), self.winfo_height(), fill="#0b1220", outline="")

        for value in range(-40, 61, 10):
            self.polyline3d([(-40, value, 0), (40, value, 0)], "#1e293b")
            self.polyline3d([(value, -20, 0), (value, 60, 0)], "#1e293b")

        self.polyline3d([(0, 0, 0), (35, 0, 0)], "#38bdf8", width=3)
        self.polyline3d([(0, 0, 0), (0, 45, 0)], "#22c55e", width=3)
        self.polyline3d([(0, 0, 0), (0, 0, 35)], "#f97316", width=3)
        self.draw_text3d((38, 0, 0), "X", "#38bdf8")
        self.draw_text3d((0, 48, 0), "Y", "#22c55e")
        self.draw_text3d((0, 0, 38), "Z", "#f97316")

        self.draw_box(-13, 13, 0, 30, 0, 20, "#ef4444")
        self.draw_text3d((0, 15, 22), "禁止区域", "#fecaca")

        points = self.arm_points()
        self.polyline3d(points, "#e2e8f0", width=4)
        for point in points:
            sx, sy = self.project(*point)
            self.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill="#f8fafc", outline="#0f172a", width=1)

        if self.pose:
            x, y, z, direction, *_ = self.pose
            sx, sy = self.project(x, y, z)
            self.create_oval(sx - 7, sy - 7, sx + 7, sy + 7, fill="#facc15", outline="#fef3c7", width=2)
            self.create_text(14, 18, anchor="w", fill="#e2e8f0", font=("Segoe UI", 10, "bold"),
                             text=f"末端: X={x:.1f} Y={y:.1f} Z={z:.1f} Dir={direction:.1f}")
        else:
            self.create_text(14, 18, anchor="w", fill="#94a3b8", font=("Segoe UI", 10, "bold"),
                             text="等待 ARM_GET_POSE 回包")


class NanoProtocolUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("NEUQ ChaseNum1 NANO 控制台")
        self.root.geometry("1320x820")
        self.root.configure(bg="#0f172a")
        self.codec = NanoCodec()
        self.ser = None
        self.reader_running = False
        self.rx_queue = queue.Queue()
        self.pressed = set()
        self.drive_active = False
        self.status_vars = {}
        self.inputs = {}

        self.setup_style()
        self.build_ui()
        self.bind_keys()
        self.root.after(40, self.poll_rx_queue)
        self.root.after(100, self.drive_loop)

    def setup_style(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#0f172a")
        style.configure("Card.TFrame", background="#162033", relief="flat")
        style.configure("TLabel", background="#0f172a", foreground="#e2e8f0", font=("Segoe UI", 10))
        style.configure("Title.TLabel", background="#0f172a", foreground="#f8fafc", font=("Segoe UI", 16, "bold"))
        style.configure("CardTitle.TLabel", background="#162033", foreground="#f8fafc", font=("Segoe UI", 11, "bold"))
        style.configure("CardText.TLabel", background="#162033", foreground="#cbd5e1", font=("Segoe UI", 10))
        style.configure("Accent.TButton", background="#0ea5e9", foreground="#f8fafc", font=("Segoe UI", 10, "bold"))
        style.configure("Danger.TButton", background="#ef4444", foreground="#fff1f2", font=("Segoe UI", 10, "bold"))
        style.configure("TButton", background="#263247", foreground="#e2e8f0", font=("Segoe UI", 10))
        style.map("TButton", background=[("active", "#334155")])
        style.map("Accent.TButton", background=[("active", "#38bdf8")])
        style.map("Danger.TButton", background=[("active", "#f87171")])
        style.configure("TEntry", fieldbackground="#0b1220", foreground="#e2e8f0", insertcolor="#e2e8f0")
        style.configure("TCombobox", fieldbackground="#0b1220", foreground="#e2e8f0")

    def card(self, parent, title):
        frame = ttk.Frame(parent, style="Card.TFrame", padding=12)
        ttk.Label(frame, text=title, style="CardTitle.TLabel").pack(anchor="w", pady=(0, 8))
        return frame

    def make_scroll_area(self, parent):
        canvas = tk.Canvas(parent, bg="#0f172a", highlightthickness=0, bd=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        content = ttk.Frame(canvas)
        window_id = canvas.create_window((0, 0), window=content, anchor="nw")

        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        def update_scroll_region(_event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def fit_content_width(event):
            canvas.itemconfigure(window_id, width=event.width)

        content.bind("<Configure>", update_scroll_region)
        canvas.bind("<Configure>", fit_content_width)
        return content, canvas

    def bind_scroll_wheel(self, widget, canvas):
        def scroll_with_wheel(event):
            step = -1 if event.delta > 0 else 1
            canvas.yview_scroll(step * 3, "units")

        widget.bind("<MouseWheel>", scroll_with_wheel, add="+")
        for child in widget.winfo_children():
            self.bind_scroll_wheel(child, canvas)

    def build_ui(self):
        top = ttk.Frame(self.root, padding=(18, 14))
        top.grid(row=0, column=0, columnspan=3, sticky="ew")
        top.columnconfigure(8, weight=1)

        ttk.Label(top, text="ChaseNum1 NANO 控制台", style="Title.TLabel").grid(row=0, column=0, padx=(0, 18))
        ttk.Label(top, text="串口").grid(row=0, column=1, padx=(0, 6))
        self.port_var = tk.StringVar(value=self.default_port())
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=14, values=self.port_list())
        self.port_box.grid(row=0, column=2, padx=(0, 10))
        ttk.Label(top, text="波特率").grid(row=0, column=3, padx=(0, 6))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(top, textvariable=self.baud_var, width=9).grid(row=0, column=4, padx=(0, 10))
        ttk.Button(top, text="连接", style="Accent.TButton", command=self.connect).grid(row=0, column=5, padx=4)
        ttk.Button(top, text="断开", command=self.disconnect).grid(row=0, column=6, padx=4)
        ttk.Button(top, text="刷新串口", command=self.refresh_ports).grid(row=0, column=7, padx=4)
        self.conn_var = tk.StringVar(value="未连接")
        ttk.Label(top, textvariable=self.conn_var).grid(row=0, column=8, sticky="e")

        left_outer = ttk.Frame(self.root, padding=(18, 0, 8, 18))
        center = ttk.Frame(self.root, padding=(8, 0, 8, 18))
        right = ttk.Frame(self.root, padding=(8, 0, 18, 18))
        left_outer.grid(row=1, column=0, sticky="nsew")
        center.grid(row=1, column=1, sticky="nsew")
        right.grid(row=1, column=2, sticky="nsew")
        self.root.columnconfigure(0, weight=0, minsize=360)
        self.root.columnconfigure(1, weight=1, minsize=520)
        self.root.columnconfigure(2, weight=0, minsize=390)
        self.root.rowconfigure(1, weight=1)

        left, left_canvas = self.make_scroll_area(left_outer)
        self.build_controls(left)
        self.bind_scroll_wheel(left, left_canvas)
        self.build_visual(center)
        self.build_status_and_log(right)

    def build_controls(self, parent):
        system = self.card(parent, "系统")
        system.pack(fill="x", pady=(0, 10))
        self.button_row(system, [
            ("HELLO", lambda: self.send(MODULE_SYSTEM, CMD_SYS_HELLO)),
            ("STATUS", lambda: self.send(MODULE_SYSTEM, CMD_SYS_GET_STATUS)),
            ("RESET", lambda: self.send(MODULE_SYSTEM, CMD_SYS_RESET)),
            ("ESTOP", lambda: self.send(MODULE_SYSTEM, CMD_SYS_ESTOP), "Danger.TButton"),
        ])

        shooter = self.card(parent, "射击")
        shooter.pack(fill="x", pady=(0, 10))
        self.button_row(shooter, [
            ("射击", lambda: self.send(MODULE_SHOOTER, CMD_SHOOTER_FIRE), "Accent.TButton"),
            ("复位", lambda: self.send(MODULE_SHOOTER, CMD_SHOOTER_RESET), "Danger.TButton"),
            ("读取", lambda: self.send(MODULE_SHOOTER, CMD_SHOOTER_GET_STATE)),
        ])

        chassis = self.card(parent, "底盘")
        chassis.pack(fill="x", pady=(0, 10))
        self.entry_grid(chassis, [
            ("键盘速度 0~100", "drive_speed", "20"),
            ("键盘角速度 0~100", "drive_wz", "30"),
        ])
        self.button_row(chassis, [
            ("STOP", lambda: self.send(MODULE_CHASSIS, CMD_CHASSIS_STOP)),
            ("刹停 BRAKE", self.send_brake, "Danger.TButton"),
            ("GET", lambda: self.send(MODULE_CHASSIS, CMD_CHASSIS_GET_STATE)),
            ("开始积分", lambda: self.send(MODULE_CHASSIS, CMD_CHASSIS_START_LOCAL_POS)),
            ("位置查询", lambda: self.send(MODULE_CHASSIS, CMD_CHASSIS_GET_LOCAL_POS)),
            ("关闭积分", lambda: self.send(MODULE_CHASSIS, CMD_CHASSIS_STOP_LOCAL_POS)),
        ])
        ttk.Label(chassis, text="方向键平移，Q/E 旋转，Space 普通停止，B 刹停", style="CardText.TLabel").pack(anchor="w", pady=(8, 0))

        run = ttk.Frame(chassis, style="Card.TFrame")
        run.pack(fill="x", pady=(10, 0))
        self.entry_grid(run, [
            ("RUN vx -100~100", "run_vx", "0"),
            ("RUN vy -100~100", "run_vy", "0"),
            ("RUN wz -100~100", "run_wz", "0"),
        ])
        ttk.Button(run, text="发送 RUN 速度", command=self.send_run).pack(fill="x", pady=(6, 0))

        rotate = ttk.Frame(chassis, style="Card.TFrame")
        rotate.pack(fill="x", pady=(10, 0))
        self.entry_grid(rotate, [("旋转角 任意", "rot_angle", "90"), ("最大角速 1~100", "rot_wz", "40")])
        ttk.Button(rotate, text="发送旋转", command=self.send_rotate).pack(fill="x", pady=(6, 0))

        translate = ttk.Frame(chassis, style="Card.TFrame")
        translate.pack(fill="x", pady=(10, 0))
        self.entry_grid(translate, [
            ("dx cm -1000~1000", "trans_dx", "20"),
            ("dy cm -1000~1000", "trans_dy", "0"),
            ("速度 1~100", "trans_speed", "20"),
        ])
        ttk.Button(translate, text="发送平移", command=self.send_translate).pack(fill="x", pady=(6, 0))

        arm = self.card(parent, "机械臂")
        arm.pack(fill="x", pady=(0, 10))
        self.entry_grid(arm, [("X", "arm_x", "20"), ("Y", "arm_y", "0"), ("Z", "arm_z", "25"), ("Dir", "arm_dir", "0")])
        self.button_row(arm, [
            ("发送目标", self.send_arm_target, "Accent.TButton"),
            ("收敛", lambda: self.send(MODULE_ARM, CMD_ARM_FOLD_HOME)),
            ("锁持", lambda: self.send(MODULE_ARM, CMD_ARM_STOP_HOLD)),
            ("读取位姿", lambda: self.send(MODULE_ARM, CMD_ARM_GET_POSE)),
        ])

        pump = self.card(parent, "气泵")
        pump.pack(fill="x", pady=(0, 10))
        self.button_row(pump, [
            ("吸附", lambda: self.send(MODULE_PUMP, CMD_PUMP_SUCK), "Accent.TButton"),
            ("放下", lambda: self.send(MODULE_PUMP, CMD_PUMP_RELEASE)),
            ("读取", lambda: self.send(MODULE_PUMP, CMD_PUMP_GET_STATE)),
        ])

    def build_visual(self, parent):
        frame = self.card(parent, "机械臂三维可视化")
        frame.pack(fill="both", expand=True)
        self.arm_view = ArmView(frame)
        self.arm_view.pack(fill="both", expand=True)

    def build_status_and_log(self, parent):
        status = self.card(parent, "解析状态")
        status.pack(fill="x", pady=(0, 10))

        refresh = ttk.Frame(status, style="Card.TFrame")
        refresh.pack(fill="x", pady=(0, 8))
        ttk.Button(refresh, text="系统", command=lambda: self.send(MODULE_SYSTEM, CMD_SYS_GET_STATUS)).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(refresh, text="底盘", command=lambda: self.send(MODULE_CHASSIS, CMD_CHASSIS_GET_STATE)).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(refresh, text="机械臂", command=lambda: self.send(MODULE_ARM, CMD_ARM_GET_POSE)).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(refresh, text="气泵", command=lambda: self.send(MODULE_PUMP, CMD_PUMP_GET_STATE)).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(refresh, text="射击", command=lambda: self.send(MODULE_SHOOTER, CMD_SHOOTER_GET_STATE)).pack(side="left", expand=True, fill="x", padx=3)

        for key, label in [
            ("system", "系统"),
            ("chassis", "底盘"),
            ("arm", "机械臂"),
            ("pump", "气泵"),
            ("shooter", "射击"),
            ("local_pos", "局部坐标"),
            ("yaw", "航向角"),
            ("tick", "下位机时间"),
        ]:
            row = ttk.Frame(status, style="Card.TFrame")
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=label, width=10, style="CardText.TLabel").pack(side="left")
            var = tk.StringVar(value="-")
            ttk.Label(row, textvariable=var, style="CardText.TLabel").pack(side="left")
            self.status_vars[key] = var
        status.bind("<Button-1>", lambda _event: self.send(MODULE_SYSTEM, CMD_SYS_GET_STATUS))

        log_card = self.card(parent, "收发信息")
        log_card.pack(fill="both", expand=True)
        self.log_text = tk.Text(log_card, bg="#0b1220", fg="#dbeafe", insertbackground="#e2e8f0",
                                height=20, relief="flat", font=("Consolas", 9), wrap="word")
        self.log_text.pack(fill="both", expand=True)
        ttk.Button(log_card, text="清空日志", command=lambda: self.log_text.delete("1.0", "end")).pack(fill="x", pady=(8, 0))

    def button_row(self, parent, buttons):
        row = ttk.Frame(parent, style="Card.TFrame")
        row.pack(fill="x", pady=4)
        for item in buttons:
            if len(item) == 2:
                text, command = item
                style = "TButton"
            else:
                text, command, style = item
            ttk.Button(row, text=text, command=command, style=style).pack(side="left", expand=True, fill="x", padx=3)

    def entry_grid(self, parent, items):
        row = ttk.Frame(parent, style="Card.TFrame")
        row.pack(fill="x", pady=3)
        for label, key, default in items:
            block = ttk.Frame(row, style="Card.TFrame")
            block.pack(side="left", expand=True, fill="x", padx=3)
            ttk.Label(block, text=label, style="CardText.TLabel").pack(anchor="w")
            var = tk.StringVar(value=default)
            ttk.Entry(block, textvariable=var, width=8).pack(fill="x")
            self.inputs[key] = var

    def bind_keys(self):
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.bind("<space>", self.on_space)
        self.root.bind("<b>", self.on_brake_key)
        self.root.bind("<B>", self.on_brake_key)

    def focus_is_text_input(self):
        widget = self.root.focus_get()
        return isinstance(widget, (ttk.Entry, ttk.Combobox, tk.Text, tk.Entry))

    def on_space(self, _event):
        if not self.focus_is_text_input():
            self.pressed.clear()
            self.drive_active = False
            self.send(MODULE_CHASSIS, CMD_CHASSIS_STOP)

    def on_brake_key(self, _event):
        if not self.focus_is_text_input():
            self.send_brake()

    def port_list(self):
        if list_ports is None:
            return []
        return [port.device for port in list_ports.comports()]

    def default_port(self):
        ports = self.port_list()
        return ports[0] if ports else "COM8"

    def refresh_ports(self):
        values = self.port_list()
        self.port_box["values"] = values
        if values:
            self.port_var.set(values[0])

    def connect(self):
        if serial is None:
            messagebox.showerror("缺少依赖", "请先安装 pyserial: pip install pyserial")
            return
        if self.ser and self.ser.is_open:
            return
        try:
            self.ser = serial.Serial(self.port_var.get(), int(self.baud_var.get()), timeout=0.03)
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))
            return
        self.reader_running = True
        threading.Thread(target=self.reader_thread, daemon=True).start()
        self.conn_var.set(f"已连接 {self.port_var.get()}")
        self.log("SYS", f"connected {self.port_var.get()} @ {self.baud_var.get()}")

    def disconnect(self):
        self.reader_running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.conn_var.set("未连接")
        self.log("SYS", "disconnected")

    def reader_thread(self):
        while self.reader_running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(256)
            except Exception as exc:
                self.rx_queue.put(("ERR", str(exc)))
                break
            if data:
                for frame in self.codec.feed(data):
                    self.rx_queue.put(("FRAME", frame))

    def send(self, module, cmd, payload=b""):
        if not self.ser or not self.ser.is_open:
            self.log("ERR", "串口未连接")
            return None
        seq, frame = self.codec.build(module, cmd, payload)
        try:
            self.ser.write(frame)
            self.ser.flush()
        except Exception as exc:
            self.log("ERR", str(exc))
            return None
        self.log("TX", f"seq={seq:02X} module={MODULE_NAMES.get(module, module)} cmd=0x{cmd:02X} data={frame.hex(' ')}")
        return seq

    def send_brake(self):
        self.pressed.clear()
        self.drive_active = False
        self.send(MODULE_CHASSIS, CMD_CHASSIS_BRAKE)

    def send_run(self):
        vx = self.get_clamped_float("run_vx", -RUN_SPEED_MAX, RUN_SPEED_MAX)
        vy = self.get_clamped_float("run_vy", -RUN_SPEED_MAX, RUN_SPEED_MAX)
        wz = self.get_clamped_float("run_wz", -RUN_WZ_MAX, RUN_WZ_MAX)
        self.send(MODULE_CHASSIS, CMD_CHASSIS_RUN, f32(vx, vy, wz))

    def send_rotate(self):
        angle = self.get_float("rot_angle")
        wz = self.get_clamped_float("rot_wz", ROT_WZ_MIN, ROT_WZ_MAX)
        self.send(MODULE_CHASSIS, CMD_CHASSIS_ROTATE_TO, f32(angle, wz))

    def send_translate(self):
        dx = self.get_clamped_float("trans_dx", -TRANS_DIST_MAX_CM, TRANS_DIST_MAX_CM)
        dy = self.get_clamped_float("trans_dy", -TRANS_DIST_MAX_CM, TRANS_DIST_MAX_CM)
        speed = self.get_clamped_float("trans_speed", TRANS_SPEED_MIN, TRANS_SPEED_MAX)
        self.send(MODULE_CHASSIS, CMD_CHASSIS_TRANSLATE_REL,
                  f32(dx, dy, speed))

    def send_arm_target(self):
        self.send(MODULE_ARM, CMD_ARM_TARGET_XYZ_DIR,
                  f32(self.get_float("arm_x"), self.get_float("arm_y"), self.get_float("arm_z"), self.get_float("arm_dir")))

    def get_float(self, key):
        try:
            return float(self.inputs[key].get())
        except ValueError:
            return 0.0

    def get_clamped_float(self, key, min_value, max_value):
        value = self.get_float(key)
        if value < min_value:
            value = min_value
        if value > max_value:
            value = max_value
        self.inputs[key].set(f"{value:g}")
        return value

    def on_key_press(self, event):
        if self.focus_is_text_input():
            return
        key = event.keysym.lower() if event.keysym in {"q", "Q", "e", "E"} else event.keysym
        if key in {"Up", "Down", "Left", "Right", "q", "e"}:
            self.pressed.add(key)

    def on_key_release(self, event):
        key = event.keysym.lower() if event.keysym in {"q", "Q", "e", "E"} else event.keysym
        if key in self.pressed:
            self.pressed.remove(key)
        if self.focus_is_text_input() and not self.drive_active:
            return
        if key in {"Up", "Down", "Left", "Right", "q", "e"} and not self.pressed and self.drive_active:
            self.drive_active = False
            if self.ser and self.ser.is_open:
                self.send(MODULE_CHASSIS, CMD_CHASSIS_STOP)

    def drive_loop(self):
        if self.pressed:
            speed = self.get_clamped_float("drive_speed", 0.0, RUN_SPEED_MAX)
            wz_speed = self.get_clamped_float("drive_wz", 0.0, RUN_WZ_MAX)
            vx = 0.0
            vy = 0.0
            wz = 0.0
            if "Up" in self.pressed:
                vy += speed
            if "Down" in self.pressed:
                vy -= speed
            if "Right" in self.pressed:
                vx += speed
            if "Left" in self.pressed:
                vx -= speed
            if "e" in self.pressed:
                wz += wz_speed
            if "q" in self.pressed:
                wz -= wz_speed
            self.drive_active = True
            if self.ser and self.ser.is_open:
                self.send(MODULE_CHASSIS, CMD_CHASSIS_RUN, f32(vx, vy, wz))
        self.root.after(100, self.drive_loop)

    def poll_rx_queue(self):
        while True:
            try:
                kind, payload = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "ERR":
                self.log("ERR", payload)
            else:
                self.handle_frame(*payload)
        self.root.after(40, self.poll_rx_queue)

    def handle_frame(self, seq, module, cmd, payload, raw):
        self.log("RX", f"seq={seq:02X} module={MODULE_NAMES.get(module, module)} resp={RESP_NAMES.get(cmd, hex(cmd))} data={raw.hex(' ')}")
        if cmd in (RESP_ACK, RESP_NACK) and len(payload) >= 5:
            ref_seq, ref_module, ref_cmd, result, state = payload[:5]
            self.log("PARSE", f"{RESP_NAMES[cmd]} ref_seq={ref_seq:02X} module={MODULE_NAMES.get(ref_module, ref_module)} cmd=0x{ref_cmd:02X} result={ERR_NAMES.get(result, result)} state={state}")
        elif cmd == RESP_EVENT and len(payload) >= 5:
            ref_seq, event_module, event_id, state, result = payload[:5]
            self.log("PARSE", f"EVENT ref_seq={ref_seq:02X} module={MODULE_NAMES.get(event_module, event_module)} event={EVENT_NAMES.get(event_id, hex(event_id))} state={state} result={ERR_NAMES.get(result, result)}")
            if event_module == MODULE_ARM:
                self.send(MODULE_ARM, CMD_ARM_GET_POSE)
            if event_module == MODULE_SYSTEM:
                self.send(MODULE_SYSTEM, CMD_SYS_GET_STATUS)
            if event_module == MODULE_SHOOTER:
                self.send(MODULE_SHOOTER, CMD_SHOOTER_GET_STATE)
        elif cmd == RESP_STATUS and len(payload) >= 16:
            self.update_status(payload)
        elif cmd == RESP_ARM_POSE and len(payload) >= 36:
            self.update_arm_pose(payload)
        elif cmd == RESP_CHASSIS_STATE and len(payload) >= 22:
            self.update_chassis_state(payload)
        elif cmd == RESP_CHASSIS_LOCAL_POS and len(payload) >= 8:
            self.update_chassis_local_pos(payload)
        elif cmd == RESP_PUMP_STATE and len(payload) >= 10:
            self.update_pump_state(payload)
        elif cmd == RESP_SHOOTER_STATE and len(payload) >= 12:
            self.update_shooter_state(payload)

    def update_status(self, payload):
        sys_s, chassis_s, arm_s, pump_s, movement, pump_on, sucked, arm_busy = payload[:8]
        tick = struct.unpack_from("<I", payload, 8)[0]
        yaw = read_f32(payload, 12)
        shooter_s = payload[16] if len(payload) >= 17 else 0
        self.status_vars["system"].set(state_name(SYS_STATES, sys_s))
        self.status_vars["chassis"].set(f"{state_name(CHASSIS_STATES, chassis_s)} / movement={movement}")
        self.status_vars["arm"].set(f"{state_name(ARM_STATES, arm_s)} / busy={arm_busy}")
        self.status_vars["pump"].set(f"{state_name(PUMP_STATES, pump_s)} / on={pump_on} sucked={sucked}")
        self.status_vars["shooter"].set(state_name(SHOOTER_STATES, shooter_s))
        self.status_vars["yaw"].set(f"{yaw:.2f} deg")
        self.status_vars["tick"].set(f"{tick} ms")

    def update_arm_pose(self, payload):
        arm_state, pose_source, arm_busy, _ = payload[:4]
        values = struct.unpack_from("<ffffffff", payload, 4)
        self.status_vars["arm"].set(f"{state_name(ARM_STATES, arm_state)} / source={pose_source} busy={arm_busy}")
        self.arm_view.set_pose(values)
        x, y, z, direction, th0, th1, th2, th3 = values
        self.log("PARSE", f"ARM pose xyz=({x:.2f},{y:.2f},{z:.2f}) dir={direction:.2f} th=({th0:.2f},{th1:.2f},{th2:.2f},{th3:.2f})")

    def update_chassis_state(self, payload):
        chassis_state, movement = payload[:2]
        yaw, vx, vy, wz, distance = struct.unpack_from("<fffff", payload, 2)
        self.status_vars["chassis"].set(f"{state_name(CHASSIS_STATES, chassis_state)} / movement={movement}")
        self.status_vars["yaw"].set(f"{yaw:.2f} deg")
        self.log("PARSE", f"CHASSIS yaw={yaw:.2f} vx={vx:.2f} vy={vy:.2f} wz={wz:.2f} dist_cm={distance:.2f}")

    def update_chassis_local_pos(self, payload):
        x_cm, y_cm = struct.unpack_from("<ff", payload, 0)
        self.status_vars["local_pos"].set(f"X={x_cm:.2f} cm / Y={y_cm:.2f} cm")
        self.log("PARSE", f"CHASSIS local_pos x={x_cm:.2f}cm y={y_cm:.2f}cm")

    def update_pump_state(self, payload):
        pump_state, pump_on, sucked, _ = payload[:4]
        freq = read_f32(payload, 4)
        strength = payload[8]
        self.status_vars["pump"].set(f"{state_name(PUMP_STATES, pump_state)} / on={pump_on} sucked={sucked}")
        self.log("PARSE", f"PUMP state={state_name(PUMP_STATES, pump_state)} on={pump_on} sucked={sucked} freq={freq:.2f} strength={strength}")

    def update_shooter_state(self, payload):
        shooter_state, friction_on, busy, _ = payload[:4]
        duty_percent = read_f32(payload, 4)
        servo_pos = struct.unpack_from("<h", payload, 8)[0]
        pwm_duty = struct.unpack_from("<H", payload, 10)[0]
        self.status_vars["shooter"].set(
            f"{state_name(SHOOTER_STATES, shooter_state)} / wheel={friction_on} busy={busy}"
        )
        self.log("PARSE", f"SHOOTER state={state_name(SHOOTER_STATES, shooter_state)} wheel={friction_on} busy={busy} duty={duty_percent:.1f}% servo={servo_pos} pwm={pwm_duty}")

    def log(self, tag, text):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{timestamp}] {tag}: {text}\n")
        self.log_text.see("end")

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    NanoProtocolUI().run()
