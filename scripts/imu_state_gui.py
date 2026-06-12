#!/usr/bin/env python3
"""
Jupiter IMU 상태 GUI
- /imu/data_raw (sensor_msgs/Imu): angular_velocity, linear_acceleration
- /imu/mag      (sensor_msgs/MagneticField): magnetic_field
roll/pitch 는 중력 벡터에서, yaw(heading) 는 자기장에 tilt 보정 적용해서 추정.

실행:
    source /opt/ros/humble/setup.bash
    python3 imu_state_gui.py
"""
import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, MagneticField


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ImuStateNode(Node):
    def __init__(self):
        super().__init__('imu_state_gui')

        # 센서 publisher 가 BEST_EFFORT 일 가능성 대비
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, qos)
        self.create_subscription(MagneticField, '/imu/mag', self.cb_mag, qos)

        self.lock = threading.Lock()
        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.mx = self.my = self.mz = 0.0
        self.imu_stamp = None
        self.mag_stamp = None
        self.imu_count = 0
        self.mag_count = 0

    def cb_imu(self, msg: Imu):
        with self.lock:
            self.ax = msg.linear_acceleration.x
            self.ay = msg.linear_acceleration.y
            self.az = msg.linear_acceleration.z
            self.gx = msg.angular_velocity.x
            self.gy = msg.angular_velocity.y
            self.gz = msg.angular_velocity.z
            self.imu_stamp = msg.header.stamp
            self.imu_count += 1

    def cb_mag(self, msg: MagneticField):
        with self.lock:
            self.mx = msg.magnetic_field.x
            self.my = msg.magnetic_field.y
            self.mz = msg.magnetic_field.z
            self.mag_stamp = msg.header.stamp
            self.mag_count += 1

    def snapshot(self):
        with self.lock:
            return dict(
                ax=self.ax, ay=self.ay, az=self.az,
                gx=self.gx, gy=self.gy, gz=self.gz,
                mx=self.mx, my=self.my, mz=self.mz,
                imu_stamp=self.imu_stamp,
                mag_stamp=self.mag_stamp,
                imu_count=self.imu_count,
                mag_count=self.mag_count,
            )


def compute_attitude(ax, ay, az, mx, my, mz):
    """가속도로 roll/pitch, 자기장으로 tilt 보정 yaw 추정 (rad)."""
    # roll: y축 회전축 기준, pitch: x축 회전축 기준 (ENU 관습)
    roll = math.atan2(ay, math.copysign(math.sqrt(az * az + 0.01 * ax * ax), az))
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

    # tilt 보정
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    mx_h = mx * cp + mz * sp
    my_h = mx * sr * sp + my * cr - mz * sr * cp
    yaw = math.atan2(-my_h, mx_h)
    return roll, pitch, yaw


class ImuGui:
    def __init__(self, node: ImuStateNode):
        self.node = node

        self.root = tk.Tk()
        self.root.title("Jupiter IMU State")
        self.root.geometry("760x560")

        style = ttk.Style()
        try:
            style.theme_use('clam')
        except tk.TclError:
            pass

        # ---------- 상단 : 자세각 큰 글씨 ----------
        top = ttk.LabelFrame(self.root, text="Attitude (deg)")
        top.pack(fill='x', padx=8, pady=6)

        self.var_roll = tk.StringVar(value="--")
        self.var_pitch = tk.StringVar(value="--")
        self.var_yaw = tk.StringVar(value="--")

        for col, (label, var) in enumerate([
            ("Roll",  self.var_roll),
            ("Pitch", self.var_pitch),
            ("Yaw",   self.var_yaw),
        ]):
            f = ttk.Frame(top)
            f.grid(row=0, column=col, padx=20, pady=8, sticky='nsew')
            ttk.Label(f, text=label, font=('TkDefaultFont', 11)).pack()
            ttk.Label(f, textvariable=var, font=('TkDefaultFont', 28, 'bold')).pack()
        for c in range(3):
            top.columnconfigure(c, weight=1)

        # ---------- 중단 : 캔버스 (attitude indicator + compass) ----------
        mid = ttk.Frame(self.root)
        mid.pack(fill='x', padx=8, pady=6)

        self.att_canvas = tk.Canvas(mid, width=240, height=240, bg='black',
                                    highlightthickness=1, highlightbackground='gray')
        self.att_canvas.pack(side='left', padx=10)

        self.cmp_canvas = tk.Canvas(mid, width=240, height=240, bg='black',
                                    highlightthickness=1, highlightbackground='gray')
        self.cmp_canvas.pack(side='left', padx=10)

        # ---------- 하단 : raw 값 ----------
        bot = ttk.Frame(self.root)
        bot.pack(fill='both', expand=True, padx=8, pady=6)

        self.var_gyro = tk.StringVar(value="--")
        self.var_acc = tk.StringVar(value="--")
        self.var_mag = tk.StringVar(value="--")
        self.var_status = tk.StringVar(value="waiting for topics...")

        for label, var in [
            ("Angular velocity [rad/s]", self.var_gyro),
            ("Linear acceleration [m/s²]", self.var_acc),
            ("Magnetic field [µT]", self.var_mag),
        ]:
            f = ttk.LabelFrame(bot, text=label)
            f.pack(fill='x', pady=3)
            ttk.Label(f, textvariable=var, font=('TkFixedFont', 11)).pack(anchor='w', padx=6, pady=2)

        ttk.Label(self.root, textvariable=self.var_status,
                  foreground='gray').pack(anchor='w', padx=10, pady=4)

        # smoothing
        self.s_roll = self.s_pitch = self.s_yaw = 0.0
        self.have_attitude = False

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.running = True
        self.last_imu_count = 0
        self.last_mag_count = 0
        self.last_rate_t = self.now()
        self.imu_hz = 0.0
        self.mag_hz = 0.0

        self.root.after(50, self.update_loop)

    @staticmethod
    def now():
        import time
        return time.monotonic()

    def update_loop(self):
        if not self.running:
            return

        s = self.node.snapshot()

        roll, pitch, yaw = compute_attitude(
            s['ax'], s['ay'], s['az'],
            s['mx'], s['my'], s['mz'],
        )

        # exponential smoothing
        a = 0.2
        if not self.have_attitude:
            self.s_roll, self.s_pitch, self.s_yaw = roll, pitch, yaw
            self.have_attitude = True
        else:
            self.s_roll = (1 - a) * self.s_roll + a * roll
            self.s_pitch = (1 - a) * self.s_pitch + a * pitch
            # yaw wrap-aware blend
            dy = math.atan2(math.sin(yaw - self.s_yaw), math.cos(yaw - self.s_yaw))
            self.s_yaw = math.atan2(math.sin(self.s_yaw + a * dy),
                                    math.cos(self.s_yaw + a * dy))

        r_deg = math.degrees(self.s_roll)
        p_deg = math.degrees(self.s_pitch)
        y_deg = math.degrees(self.s_yaw)
        if y_deg < 0:
            y_deg += 360.0

        self.var_roll.set(f"{r_deg:+6.1f}°")
        self.var_pitch.set(f"{p_deg:+6.1f}°")
        self.var_yaw.set(f"{y_deg:6.1f}°")

        self.var_gyro.set(
            f"x={s['gx']:+8.4f}   y={s['gy']:+8.4f}   z={s['gz']:+8.4f}"
        )
        self.var_acc.set(
            f"x={s['ax']:+8.3f}   y={s['ay']:+8.3f}   z={s['az']:+8.3f}"
        )
        self.var_mag.set(
            f"x={s['mx']*1e6:+8.2f}   y={s['my']*1e6:+8.2f}   z={s['mz']*1e6:+8.2f}"
            f"   |B|={math.sqrt(s['mx']**2+s['my']**2+s['mz']**2)*1e6:7.2f}"
        )

        # rate
        t = self.now()
        if t - self.last_rate_t >= 1.0:
            dt = t - self.last_rate_t
            self.imu_hz = (s['imu_count'] - self.last_imu_count) / dt
            self.mag_hz = (s['mag_count'] - self.last_mag_count) / dt
            self.last_imu_count = s['imu_count']
            self.last_mag_count = s['mag_count']
            self.last_rate_t = t

        ok_imu = s['imu_count'] > 0
        ok_mag = s['mag_count'] > 0
        self.var_status.set(
            f"/imu/data_raw : {'OK' if ok_imu else 'NO DATA'}  ({self.imu_hz:5.1f} Hz, n={s['imu_count']})   "
            f"/imu/mag : {'OK' if ok_mag else 'NO DATA'}  ({self.mag_hz:5.1f} Hz, n={s['mag_count']})"
        )

        self.draw_attitude(self.s_roll, self.s_pitch)
        self.draw_compass(self.s_yaw)

        self.root.after(50, self.update_loop)

    def draw_attitude(self, roll, pitch):
        c = self.att_canvas
        c.delete('all')
        w = int(c['width']); h = int(c['height'])
        cx, cy = w / 2, h / 2
        r = min(w, h) / 2 - 8

        # horizon: pitch shifts vertically, roll rotates
        pitch_px = clamp(math.degrees(pitch) / 45.0, -1.0, 1.0) * r * 0.6
        cr, sr = math.cos(-roll), math.sin(-roll)

        # sky / ground rectangle, drawn as rotated polygon
        big = r * 2.5

        def rot(x, y):
            return (cx + x * cr - y * sr, cy + x * sr + y * cr)

        sky = [rot(-big, -big), rot(big, -big),
               rot(big, pitch_px), rot(-big, pitch_px)]
        ground = [rot(-big, pitch_px), rot(big, pitch_px),
                  rot(big, big), rot(-big, big)]

        c.create_polygon([v for p in sky for v in p], fill='#1e6fd1', outline='')
        c.create_polygon([v for p in ground for v in p], fill='#8a5a2b', outline='')

        # horizon line
        h1 = rot(-r, pitch_px); h2 = rot(r, pitch_px)
        c.create_line(h1[0], h1[1], h2[0], h2[1], fill='white', width=2)

        # fixed aircraft symbol
        c.create_line(cx - 50, cy, cx - 15, cy, fill='yellow', width=3)
        c.create_line(cx + 15, cy, cx + 50, cy, fill='yellow', width=3)
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill='yellow', outline='yellow')

        # bezel
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline='gray', width=2)
        c.create_text(cx, 12, text="Attitude", fill='white')

    def draw_compass(self, yaw):
        c = self.cmp_canvas
        c.delete('all')
        w = int(c['width']); h = int(c['height'])
        cx, cy = w / 2, h / 2
        r = min(w, h) / 2 - 12

        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline='gray', width=2)

        # cardinal: rotate by -yaw so that current heading is up
        labels = [('N', 0), ('E', 90), ('S', 180), ('W', 270)]
        for lab, deg in labels:
            ang = math.radians(deg) - yaw - math.pi / 2  # 0deg=up
            x = cx + (r - 16) * math.cos(ang)
            y = cy + (r - 16) * math.sin(ang)
            color = 'red' if lab == 'N' else 'white'
            c.create_text(x, y, text=lab, fill=color,
                          font=('TkDefaultFont', 12, 'bold'))

        # tick marks every 30deg
        for deg in range(0, 360, 30):
            ang = math.radians(deg) - yaw - math.pi / 2
            x1 = cx + (r - 4) * math.cos(ang)
            y1 = cy + (r - 4) * math.sin(ang)
            x2 = cx + r * math.cos(ang)
            y2 = cy + r * math.sin(ang)
            c.create_line(x1, y1, x2, y2, fill='gray')

        # fixed heading indicator (up)
        c.create_polygon(cx, cy - r + 2,
                         cx - 8, cy - r + 18,
                         cx + 8, cy - r + 18,
                         fill='yellow', outline='yellow')

        deg = math.degrees(yaw)
        if deg < 0:
            deg += 360
        c.create_text(cx, cy, text=f"{deg:5.1f}°",
                      fill='white', font=('TkDefaultFont', 14, 'bold'))
        c.create_text(cx, 12, text="Heading (mag)", fill='white')

    def on_close(self):
        self.running = False
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def spin_ros(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    rclpy.init()
    node = ImuStateNode()

    t = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    t.start()

    gui = ImuGui(node)
    try:
        gui.root.mainloop()
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    main()
