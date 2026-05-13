import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import struct
import numpy as np
import threading
import time
import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

# 解决中文显示乱码问题
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False


# =====================================================================
# ⚙️ 机械臂物理参数统一配置区
# =====================================================================
class ArmParams:
    H0 = 8.8  # 云台高度
    H1 = 6.5  # 齿轮相对云台高度
    R = 3.0  # 大齿轮投影中心距
    L1 = 17.5  # 臂1长度
    C = 2.5  # 舵机2偏置距离
    L2 = 20.5  # 臂2长度
    L_suc = 11.0  # 第四舵机轴心到吸盘末端长度

    @classmethod
    def get_derived(cls):
        L_eff = np.sqrt(cls.L1 ** 2 + cls.C ** 2)
        phi = np.arctan2(cls.C, cls.L1)
        L_max = cls.R + cls.L1 + cls.L2 + cls.L_suc
        return L_eff, phi, L_max


class ObstacleParams:
    ENABLE = True
    ROD_X = 0.0
    ROD_Y = 30.0
    ROD_RADIUS = 1.5
    X_SIDE_EPS = 1.0
    FOLD_TH1 = 80.0
    FOLD_TH2 = -145.0
    FOLD_TH3 = 75.0
    S3_MIN = -180.0
    S3_MAX = 180.0


# =====================================================================
# 🧮 运动学核心算法
# =====================================================================
def normalize_deg(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def world_x_side(world_x):
    if world_x > ObstacleParams.X_SIDE_EPS:
        return 1
    if world_x < -ObstacleParams.X_SIDE_EPS:
        return -1
    return 0


def is_target_in_front_obstacle(world_x, world_y, world_z):
    if not ObstacleParams.ENABLE:
        return False
    return (
        abs(world_x - ObstacleParams.ROD_X) <= ObstacleParams.ROD_RADIUS
        and abs(world_y - ObstacleParams.ROD_Y) <= ObstacleParams.ROD_RADIUS
    )


def is_servo_angle_in_range(th0, th1, th2, th3):
    actual_s0 = th0 + 5.0
    actual_s1 = th1 * 3.0 - 533.0
    actual_s2 = -th2 - 13.0
    actual_s3 = -th3 - 94.0

    return (
        -180.0 <= actual_s0 <= 180.0
        and -533.0 <= actual_s1 <= 7.0
        and -90.0 <= actual_s2 <= 150.0
        and ObstacleParams.S3_MIN <= actual_s3 <= ObstacleParams.S3_MAX
    )


def inverse_kinematics_wrist_2d(plane_x, plane_z):
    L_eff, phi, _ = ArmParams.get_derived()
    dx = plane_x - ArmParams.R
    dz = plane_z - (ArmParams.H0 + ArmParams.H1)
    D = np.sqrt(dx ** 2 + dz ** 2)

    if D < 1e-6:
        return None, None

    if D > (L_eff + ArmParams.L2) or D < abs(L_eff - ArmParams.L2):
        return None, None

    beta = np.arccos(np.clip((L_eff ** 2 + D ** 2 - ArmParams.L2 ** 2) / (2 * L_eff * D), -1.0, 1.0))
    alpha = np.arctan2(dz, dx)
    theta_eff = alpha + beta
    theta1_rad = theta_eff - phi
    gamma = np.arccos(np.clip((L_eff ** 2 + ArmParams.L2 ** 2 - D ** 2) / (2 * L_eff * ArmParams.L2), -1.0, 1.0))
    theta2_rad = gamma - np.pi + phi

    return np.degrees(theta1_rad), np.degrees(theta2_rad)


def inverse_kinematics_plane(th0, plane_x, plane_z, suc_dir_deg):
    dir_rad = np.radians(suc_dir_deg)
    wrist_x = plane_x - ArmParams.L_suc * np.sin(dir_rad)
    wrist_z = plane_z + ArmParams.L_suc * np.cos(dir_rad)
    th1, th2 = inverse_kinematics_wrist_2d(wrist_x, wrist_z)

    if th1 is None:
        return None

    suc_std_dir = normalize_deg(suc_dir_deg - 90.0)
    th3 = normalize_deg(suc_std_dir - th1 - th2)
    th0 = normalize_deg(th0)

    if not is_servo_angle_in_range(th0, th1, th2, th3):
        return None

    return th0, th1, th2, th3


def inverse_kinematics_target(world_x, world_y, world_z, suc_dir_deg):
    plane_x = np.hypot(world_x, world_y)
    th0 = 0.0
    if plane_x > 1e-6:
        th0 = np.degrees(np.arctan2(world_x, world_y))
    return inverse_kinematics_plane(th0, plane_x, world_z, suc_dir_deg)


def forward_kinematics_3d(th0, th1, th2, th3):
    r0, r1, r2, r3 = np.radians([th0, th1, th2, th3])
    v_xy = np.array([np.sin(r0), np.cos(r0), 0])
    v_z = np.array([0, 0, 1])

    P0 = np.array([0, 0, 0])
    P1 = np.array([0, 0, ArmParams.H0])
    P2 = ArmParams.R * v_xy + (ArmParams.H0 + ArmParams.H1) * v_z
    P3 = P2 + ArmParams.L1 * (np.cos(r1) * v_xy + np.sin(r1) * v_z)
    P4 = P3 + ArmParams.C * (-np.sin(r1) * v_xy + np.cos(r1) * v_z)
    P5 = P4 + ArmParams.L2 * (np.cos(r1 + r2) * v_xy + np.sin(r1 + r2) * v_z)
    P6 = P5 + ArmParams.L_suc * (np.cos(r1 + r2 + r3) * v_xy + np.sin(r1 + r2 + r3) * v_z)

    return P0, P1, P2, P3, P4, P5, P6


def folded_target_for_plane_angle(plane_angle_rad):
    th0 = np.degrees(plane_angle_rad)
    return forward_kinematics_3d(th0, ObstacleParams.FOLD_TH1, ObstacleParams.FOLD_TH2, ObstacleParams.FOLD_TH3)


# =====================================================================
# 🖥️ GUI 控制台主程序
# =====================================================================
class ArmControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("机械臂数字孪生与串口分析终端 - Zhuabao")
        self.root.geometry("1200x850")  # 稍微调高以容纳日志区
        self.root.configure(bg='#1e1e1e')

        self.serial_port = None
        self.is_receiving = False
        self.last_sent_x = None
        self.style_setup()
        self.create_widgets()
        self.init_plot()
        self.update_kinematics()

    def style_setup(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TFrame", background='#1e1e1e')
        style.configure("Panel.TFrame", background='#2d2d2d')
        style.configure("TLabel", background='#2d2d2d', foreground='#eeeeee', font=('Microsoft YaHei', 10))
        style.configure("Header.TLabel", font=('Microsoft YaHei', 11, 'bold'), foreground='#00e5ff')
        style.configure("TButton", font=('Microsoft YaHei', 10, 'bold'))
        style.configure("Send.TButton", background='#007acc', foreground='white')
        style.configure("Stop.TButton", background='#d32f2f', foreground='white')

    def create_widgets(self):
        # 左侧控制面板
        left_frame = ttk.Frame(self.root, style="Panel.TFrame")
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        # 1. 串口区域
        serial_frame = ttk.Frame(left_frame, style="Panel.TFrame")
        serial_frame.pack(fill=tk.X, pady=5, padx=10)
        ttk.Label(serial_frame, text="🔌 通信连接", style="Header.TLabel").pack(anchor='w')

        port_sub = ttk.Frame(serial_frame, style="Panel.TFrame")
        port_sub.pack(fill=tk.X, pady=5)
        self.cb_ports = ttk.Combobox(port_sub, width=12)
        self.refresh_ports()
        self.cb_ports.pack(side=tk.LEFT, padx=(0, 5))
        self.btn_connect = ttk.Button(port_sub, text="连接", width=8, command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT)

        # 2. 滑块控制区
        ctrl_frame = ttk.Frame(left_frame, style="Panel.TFrame")
        ctrl_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(ctrl_frame, text="🎛️ 运动控制", style="Header.TLabel").pack(anchor='w')

        _, _, L_max = ArmParams.get_derived()
        self.vars = {'x': tk.DoubleVar(value=0.0), 'y': tk.DoubleVar(value=30.0),
                     'z': tk.DoubleVar(value=18.0), 'dir': tk.DoubleVar(value=0.0)}

        configs = [('x', '末端世界X', -L_max, L_max), ('y', '末端世界Y', -L_max, L_max),
                   ('z', '末端世界Z', 0, L_max), ('dir', '竖直平面吸盘方向', 0, 360)]

        for key, label, vmin, vmax in configs:
            sub = ttk.Frame(ctrl_frame, style="Panel.TFrame")
            sub.pack(fill=tk.X, pady=1)
            ttk.Label(sub, text=label).pack(side=tk.LEFT)
            val_lbl = ttk.Label(sub, text="0.0", width=6, anchor='e', foreground='#ffb000')
            val_lbl.pack(side=tk.RIGHT)

            slider = ttk.Scale(ctrl_frame, from_=vmin, to=vmax, variable=self.vars[key], orient='horizontal',
                               command=lambda val, k=key, l=val_lbl: self.on_slider_move(k, l))
            slider.pack(fill=tk.X, pady=(0, 5))
            val_lbl.configure(text=f"{self.vars[key].get():.1f}")

        # 3. 信息看板
        info_frame = ttk.Frame(left_frame, style="Panel.TFrame")
        info_frame.pack(fill=tk.X, padx=10, pady=5)
        self.lbl_status = ttk.Label(info_frame, text="解算正常", foreground="#00ffcc", font=('Consolas', 10))
        self.lbl_status.pack(anchor='w')

        # 4. 按钮区
        btn_frame = ttk.Frame(left_frame, style="Panel.TFrame")
        btn_frame.pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(btn_frame, text="🚀 发送任务", style="Send.TButton", command=self.send_task_data).pack(fill=tk.X,
                                                                                                         pady=2)
        ttk.Button(btn_frame, text="🛑 紧急停止", style="Stop.TButton", command=self.send_emergency_stop).pack(fill=tk.X,
                                                                                                              pady=2)

        # 5. 通讯日志区 (新增)
        log_frame = ttk.Frame(left_frame, style="Panel.TFrame")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        ttk.Label(log_frame, text="📜 通讯日志 (TX/RX)", style="Header.TLabel").pack(anchor='w')

        self.log_area = scrolledtext.ScrolledText(log_frame, width=35, height=15,
                                                  bg='#121212', fg='#dcdcdc',
                                                  font=('Consolas', 9), padx=5, pady=5)
        self.log_area.pack(fill=tk.BOTH, expand=True, pady=5)
        self.log_area.tag_config('tx', foreground='#00e5ff')
        self.log_area.tag_config('rx', foreground='#ffeb3b')
        self.log_area.tag_config('err', foreground='#ff5252')

        # 右侧绘图区
        self.plot_frame = tk.Frame(self.root, bg='#121212')
        self.plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    def log_message(self, msg, tag='info'):
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.log_area.insert(tk.END, f"[{timestamp}] {msg}\n", tag)
        self.log_area.see(tk.END)

    def init_plot(self):
        self.fig = plt.Figure(figsize=(7, 6), facecolor='#121212')
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#121212')
        for axis in [self.ax.xaxis, self.ax.yaxis, self.ax.zaxis]:
            axis.set_pane_color((0.15, 0.15, 0.15, 1.0))
            axis.line.set_color("#444444")
        self.ax.tick_params(colors='#888888')
        _, _, L_max = ArmParams.get_derived()
        self.ax.set_xlim([-L_max, L_max]);
        self.ax.set_ylim([-L_max, L_max]);
        self.ax.set_zlim([0, L_max])
        self.ax.set_xlabel('X/cm', color='#aaaaaa')
        self.ax.set_ylabel('Y/cm', color='#aaaaaa')
        self.ax.set_zlabel('Z/cm', color='#aaaaaa')
        self.line_base, = self.ax.plot([], [], [], color='#555555', lw=6)
        self.line_arm, = self.ax.plot([], [], [], color='#00F0FF', lw=4, marker='o', markersize=6, markerfacecolor='w')
        self.line_suc, = self.ax.plot([], [], [], color='#FF0055', lw=3)
        self.line_rod, = self.ax.plot(
            [ObstacleParams.ROD_X, ObstacleParams.ROD_X],
            [ObstacleParams.ROD_Y, ObstacleParams.ROD_Y],
            [0, L_max],
            color='#ff5252',
            lw=2,
            linestyle='--'
        )
        self.scatter_fold = self.ax.scatter(
            [0],
            [22.0],
            [15.0],
            color='#66ff66',
            s=40,
            marker='o',
            label='折叠参考'
        )
        self.scatter_target = self.ax.scatter([], [], [], color='#FFB000', s=100, marker='x')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.cb_ports['values'] = ports
        if ports: self.cb_ports.current(0)

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.is_receiving = False
            self.serial_port.close()
            self.btn_connect.configure(text="连接")
            self.log_message("❌ 串口已断开", 'err')
        else:
            port = self.cb_ports.get()
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=0.1)
                self.btn_connect.configure(text="断开")
                self.is_receiving = True
                threading.Thread(target=self.receive_handler, daemon=True).start()
                self.log_message(f"✅ 已连接至 {port}")
            except Exception as e:
                messagebox.showerror("错误", f"连接失败: {e}")

    def receive_handler(self):
        """后台线程：处理来自单片机的数据回传 (支持字符串解析)"""
        buffer = ""
        while self.is_receiving:
            if self.serial_port and self.serial_port.in_waiting > 0:
                try:
                    # 读取字节数据
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    # 尝试用 utf-8 解码成普通字符串，忽略无法解析的乱码字节
                    text = data.decode('utf-8', errors='ignore')
                    buffer += text

                    # 按行分割，保证日志打印的完整性
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()  # 去除回车换行符
                        if line:
                            # 判断单片机发来的字符串类型并用不同颜色高亮
                            if "SUCCESS" in line:
                                self.root.after(0, self.log_message, f"RX: {line}", 'rx')
                            elif "ERROR" in line:
                                self.root.after(0, self.log_message, f"RX: {line}", 'err')
                            elif "STOP" in line:
                                self.root.after(0, self.log_message, f"RX: {line}", 'err')
                            else:
                                self.root.after(0, self.log_message, f"RX: {line}", 'rx')
                except Exception as e:
                    pass
            time.sleep(0.01)

    def on_slider_move(self, key, label_widget):
        val = self.vars[key].get()
        label_widget.configure(text=f"{val:.1f}")
        self.update_kinematics()

    def update_kinematics(self):
        target_world_x = self.vars['x'].get()
        target_world_y = self.vars['y'].get()
        suc_dir = self.vars['dir'].get()
        target_world_z = self.vars['z'].get()
        target_plane_angle = np.arctan2(target_world_x, target_world_y) if np.hypot(target_world_x, target_world_y) > 1e-6 else 0.0
        _, _, _, _, _, _, fold_tip = folded_target_for_plane_angle(target_plane_angle)
        result = inverse_kinematics_target(target_world_x, target_world_y, target_world_z, suc_dir)
        in_obstacle = is_target_in_front_obstacle(target_world_x, target_world_y, target_world_z)
        self.scatter_fold._offsets3d = ([fold_tip[0]], [fold_tip[1]], [fold_tip[2]])

        if in_obstacle:
            self.lbl_status.configure(text="⚠️ 目标太靠近前方细杆", foreground="#ff3333")
            self.line_arm.set_data_3d([], [], [])
            self.line_suc.set_data_3d([], [], [])
            self.scatter_target._offsets3d = ([target_world_x], [target_world_y], [target_world_z])
        elif result is None:
            self.lbl_status.configure(text="⚠️ 目标无法触达", foreground="#ff3333")
            self.line_arm.set_data_3d([], [], [])
            self.line_suc.set_data_3d([], [], [])
            self.scatter_target._offsets3d = ([target_world_x], [target_world_y], [target_world_z])
        else:
            th0, th1, th2, th3 = result
            motion_hint = "直达"
            if self.last_sent_x is not None:
                last_side = world_x_side(self.last_sent_x)
                target_side = world_x_side(target_world_x)
                if last_side != 0 and target_side != 0 and last_side != target_side:
                    motion_hint = "跨零折叠"
            self.lbl_status.configure(
                text=f"{motion_hint} | th0:{th0:4.1f}° | th1:{th1:4.1f}° | th2:{th2:4.1f}° | th3:{th3:4.1f}°",
                foreground="#00ffcc"
            )
            P0, P1, P2, P3, P4, P5, P6 = forward_kinematics_3d(th0, th1, th2, th3)
            self.line_base.set_data_3d([P0[0], P1[0], P2[0]], [P0[1], P1[1], P2[1]], [P0[2], P1[2], P2[2]])
            self.line_arm.set_data_3d([P2[0], P3[0], P4[0], P5[0]], [P2[1], P3[1], P4[1], P5[1]],
                                      [P2[2], P3[2], P4[2], P5[2]])
            self.line_suc.set_data_3d([P5[0], P6[0]], [P5[1], P6[1]], [P5[2], P6[2]])
            self.scatter_target._offsets3d = ([target_world_x], [target_world_y], [target_world_z])
        self.canvas.draw_idle()

    def send_packet(self, cmd_type, payload=b''):
        if not self.serial_port or not self.serial_port.is_open:
            self.log_message("⚠️ 发送失败：串口未连接", 'err')
            return False

        header, tail = b'\x5A\xA5', b'\x0D\x0A'
        checksum = (cmd_type + sum(payload)) & 0xFF
        packet = header + struct.pack('B', cmd_type) + payload + struct.pack('B', checksum) + tail
        self.serial_port.write(packet)

        # 发送端日志：同时显示 HEX 和 可读信息
        if cmd_type == 0x01:
            self.log_message(
                f"TX (任务): X={self.vars['x'].get():.1f}, Y={self.vars['y'].get():.1f}, "
                f"Z={self.vars['z'].get():.1f}, Dir={self.vars['dir'].get():.1f}",
                'tx'
            )
        elif cmd_type == 0x00:
            self.log_message(f"TX (急停): 🛑 触发全局急停指令!", 'err')
        return True

    def send_task_data(self):
        if is_target_in_front_obstacle(self.vars['x'].get(), self.vars['y'].get(), self.vars['z'].get()):
            self.log_message("TX blocked: target is too close to the front rod", 'err')
            return
        if inverse_kinematics_target(self.vars['x'].get(), self.vars['y'].get(), self.vars['z'].get(),
                                     self.vars['dir'].get()) is None:
            self.log_message("TX blocked: target is unreachable or angle-limited", 'err')
            return

        payload = struct.pack('<ffff', self.vars['x'].get(), self.vars['y'].get(), self.vars['z'].get(),
                              self.vars['dir'].get())
        if self.send_packet(0x01, payload):
            self.last_sent_x = self.vars['x'].get()
            self.update_kinematics()

    def send_emergency_stop(self):
        self.send_packet(0x00)


if __name__ == "__main__":
    root = tk.Tk()
    app = ArmControllerGUI(root)
    root.mainloop()
