import ctypes
import os
import subprocess
import sys
import time
import numpy as np
import airsim
from ctypes import wintypes

from PySide6.QtCore import Qt, QTimer, QThread, Signal, Slot
from PySide6.QtGui import QVector3D
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QStatusBar, QMessageBox,
    QSplitter
)
import pyqtgraph.opengl as gl

PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIMULATOR = os.path.join(
    PROJECT, "AirSimNH", "AirSimNH",
    "WindowsNoEditor", "AirSimNH.exe"
)

# --- Win32 Window Embedding APIs ---
user32 = ctypes.WinDLL("user32", use_last_error=True)

EnumWindowsProc = ctypes.WINFUNCTYPE(
    wintypes.BOOL, wintypes.HWND, wintypes.LPARAM
)

user32.EnumWindows.argtypes = [EnumWindowsProc, wintypes.LPARAM]
user32.EnumWindows.restype = wintypes.BOOL
user32.GetWindowThreadProcessId.argtypes = [
    wintypes.HWND, ctypes.POINTER(wintypes.DWORD)
]
user32.GetWindowThreadProcessId.restype = wintypes.DWORD
user32.IsWindowVisible.argtypes = [wintypes.HWND]
user32.IsWindowVisible.restype = wintypes.BOOL
user32.GetWindowTextLengthW.argtypes = [wintypes.HWND]
user32.GetWindowTextLengthW.restype = ctypes.c_int
user32.GetWindowTextW.argtypes = [
    wintypes.HWND, wintypes.LPWSTR, ctypes.c_int
]
user32.GetWindowTextW.restype = ctypes.c_int
user32.SetParent.argtypes = [wintypes.HWND, wintypes.HWND]
user32.SetParent.restype = wintypes.HWND
user32.SetWindowPos.argtypes = [
    wintypes.HWND, wintypes.HWND, ctypes.c_int, ctypes.c_int,
    ctypes.c_int, ctypes.c_int, wintypes.UINT
]
user32.SetWindowPos.restype = wintypes.BOOL
user32.ShowWindow.argtypes = [wintypes.HWND, ctypes.c_int]
user32.ShowWindow.restype = wintypes.BOOL
user32.IsWindow.argtypes = [wintypes.HWND]
user32.IsWindow.restype = wintypes.BOOL

get_style = user32.GetWindowLongPtrW
set_style = user32.SetWindowLongPtrW
get_style.argtypes = [wintypes.HWND, ctypes.c_int]
get_style.restype = ctypes.c_ssize_t
set_style.argtypes = [wintypes.HWND, ctypes.c_int, ctypes.c_ssize_t]
set_style.restype = ctypes.c_ssize_t

GWL_STYLE = -16
WS_CHILD = 0x40000000
WS_POPUP = 0x80000000
WS_CAPTION = 0x00C00000
WS_THICKFRAME = 0x00040000
WS_MINIMIZEBOX = 0x00020000
WS_MAXIMIZEBOX = 0x00010000
WS_SYSMENU = 0x00080000
SW_RESTORE = 9
SWP_NOZORDER = 0x0004
SWP_FRAMECHANGED = 0x0020
SWP_SHOWWINDOW = 0x0040


def find_window_for_pid(pid):
    candidates = []

    @EnumWindowsProc
    def callback(hwnd, lparam):
        if not user32.IsWindowVisible(hwnd):
            return True

        length = user32.GetWindowTextLengthW(hwnd)
        if length <= 0:
            return True

        title_buffer = ctypes.create_unicode_buffer(length + 1)
        user32.GetWindowTextW(hwnd, title_buffer, length + 1)
        title = title_buffer.value

        if "AirSimNH" not in title:
            return True

        window_pid = wintypes.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(window_pid))
        candidates.append((int(hwnd), window_pid.value, title))
        return True

    user32.EnumWindows(callback, 0)

    for hwnd, window_pid, title in candidates:
        if window_pid == pid:
            return hwnd

    if candidates:
        hwnd, window_pid, title = candidates[0]
        return hwnd

    return None


class Viewport(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("viewport")
        self.setMinimumSize(480, 320)
        self.hosted_hwnd = None
        self.on_resize = None

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.hosted_hwnd and self.on_resize:
            self.on_resize()


# --- Spatial Transformations ---

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    norm = (w * w + x * x + y * y + z * z) ** 0.5
    if norm > 1e-6:
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
    else:
        w, x, y, z = 1.0, 0.0, 0.0, 0.0
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]
    ], dtype=np.float32)


def transform_sensor_to_world_vis(pts_sensor, sensor_pose, vehicle_pose):
    if len(pts_sensor) == 0:
        return np.empty((0, 3), dtype=np.float32)

    # 1. Sensor body to vehicle body frame
    Rs = quaternion_to_rotation_matrix(sensor_pose.orientation)
    Ts = np.array([
        sensor_pose.position.x_val,
        sensor_pose.position.y_val,
        sensor_pose.position.z_val
    ], dtype=np.float32)
    pts_drone = pts_sensor @ Rs.T + Ts

    # 2. Vehicle body to world NED frame
    Rv = quaternion_to_rotation_matrix(vehicle_pose.orientation)
    Tv = np.array([
        vehicle_pose.position.x_val,
        vehicle_pose.position.y_val,
        vehicle_pose.position.z_val
    ], dtype=np.float32)
    pts_world_ned = pts_drone @ Rv.T + Tv

    # 3. Convert AirSim NED (X North, Y East, Z Down) to Visualization (X North, Y East, Z Up)
    pts_vis = pts_world_ned.copy()
    pts_vis[:, 2] = -pts_world_ned[:, 2]
    return pts_vis


def compute_point_colors(pts_vis):
    if len(pts_vis) == 0:
        return np.empty((0, 4), dtype=np.float32)

    z = pts_vis[:, 2]
    z_min, z_max = np.min(z), np.max(z)
    if z_max - z_min < 1e-3:
        norm_z = np.zeros_like(z)
    else:
        norm_z = np.clip((z - z_min) / (z_max - z_min), 0.0, 1.0)

    colors = np.zeros((len(pts_vis), 4), dtype=np.float32)
    colors[:, 0] = 0.1 + 0.8 * norm_z
    colors[:, 1] = 0.5 + 0.5 * (1.0 - np.abs(norm_z - 0.5) * 2)
    colors[:, 2] = 1.0 - 0.7 * norm_z
    colors[:, 3] = 0.85
    return colors


# --- Workers ---

class LidarTelemetryWorker(QThread):
    lidar_data_signal = Signal(object, str, int, float, float, float)
    telemetry_signal = Signal(dict)
    sensor_status_signal = Signal(str, str, int)
    connection_status_signal = Signal(bool, str)

    def __init__(self, ip="127.0.0.1", port=41451):
        super().__init__()
        self.ip = ip
        self.port = port
        self.selected_vehicle = "Drone1"
        self.running = False
        self.client = None

    def set_vehicle(self, vehicle_name):
        self.selected_vehicle = vehicle_name

    def stop(self):
        self.running = False
        self.wait(2000)

    def run(self):
        self.running = True
        try:
            self.client = airsim.MultirotorClient(
                ip=self.ip, port=self.port, timeout_value=3
            )
            self.client.confirmConnection()
            vehicles = self.client.listVehicles()
            if vehicles and self.selected_vehicle not in vehicles:
                self.selected_vehicle = vehicles[0]
            self.connection_status_signal.emit(
                True, f"Connected to AirSim API ({len(vehicles)} vehicle: {', '.join(vehicles)})"
            )
        except Exception as exc:
            self.connection_status_signal.emit(
                False, f"API Connection failed: {exc}"
            )
            self.running = False
            return

        while self.running:
            try:
                state = self.client.getMultirotorState(
                    vehicle_name=self.selected_vehicle
                )
                pos = state.kinematics_estimated.position
                vel = state.kinematics_estimated.linear_velocity
                dx_vis, dy_vis, dz_vis = pos.x_val, pos.y_val, -pos.z_val

                vehicles_list = self.client.listVehicles()
                telemetry = {
                    "active_vehicles": vehicles_list,
                    "pos_x": pos.x_val,
                    "pos_y": pos.y_val,
                    "altitude": -pos.z_val,
                    "vel_x": vel.x_val,
                    "vel_y": vel.y_val,
                    "vel_z": vel.z_val,
                    "ready": state.ready,
                    "can_arm": state.can_arm,
                    "landed_state": str(state.landed_state),
                    "vehicle": self.selected_vehicle
                }
                self.telemetry_signal.emit(telemetry)

                # Fetch LiDAR Data
                lidar_data = self.client.getLidarData(
                    lidar_name="Lidar1", vehicle_name=self.selected_vehicle
                )

                got_points = False
                if lidar_data and len(lidar_data.point_cloud) >= 3:
                    pts_raw = np.array(
                        lidar_data.point_cloud, dtype=np.float32
                    ).reshape(-1, 3)
                    valid_mask = np.isfinite(pts_raw).all(axis=1)
                    pts_valid = pts_raw[valid_mask]

                    if len(pts_valid) > 0:
                        got_points = True
                        pts_vis = transform_sensor_to_world_vis(
                            pts_valid, lidar_data.pose, state.kinematics_estimated
                        )
                        self.lidar_data_signal.emit(
                            pts_vis, "Lidar1", len(pts_vis),
                            dx_vis, dy_vis, dz_vis
                        )
                        self.sensor_status_signal.emit(
                            "ACTIVE", "Lidar1", len(pts_vis)
                        )

                # Depth Camera Fallback if LiDAR unavailable
                if not got_points:
                    responses = self.client.simGetImages([
                        airsim.ImageRequest(
                            "0", airsim.ImageType.DepthPerspective, True, False
                        )
                    ], vehicle_name=self.selected_vehicle)

                    if responses and len(responses) > 0 and responses[0].width > 0:
                        img_data = np.array(
                            responses[0].image_data_float, dtype=np.float32
                        )
                        w, h = responses[0].width, responses[0].height
                        if len(img_data) == w * h:
                            depth_img = img_data.reshape(h, w)
                            step = 8
                            sub_depth = depth_img[::step, ::step]

                            fov_deg = 90.0
                            f_len = (w / 2.0) / np.tan(np.radians(fov_deg / 2.0))

                            u_idx = np.arange(0, w, step, dtype=np.float32)
                            v_idx = np.arange(0, h, step, dtype=np.float32)
                            uu, vv = np.meshgrid(u_idx, v_idx)

                            valid_depth = (
                                (sub_depth > 0.2) &
                                (sub_depth < 50.0) &
                                np.isfinite(sub_depth)
                            )

                            if np.any(valid_depth):
                                got_points = True
                                z_c = sub_depth[valid_depth]
                                x_c = (uu[valid_depth] - w / 2.0) * z_c / f_len
                                y_c = (vv[valid_depth] - h / 2.0) * z_c / f_len

                                pts_cam = np.column_stack([z_c, x_c, y_c])
                                cam_pose = airsim.Pose(
                                    responses[0].camera_position,
                                    responses[0].camera_orientation
                                )
                                pts_vis = transform_sensor_to_world_vis(
                                    pts_cam, cam_pose, state.kinematics_estimated
                                )
                                self.lidar_data_signal.emit(
                                    pts_vis, "DepthCamera", len(pts_vis),
                                    dx_vis, dy_vis, dz_vis
                                )
                                self.sensor_status_signal.emit(
                                    "ACTIVE", "Depth Camera", len(pts_vis)
                                )

                if not got_points:
                    self.sensor_status_signal.emit(
                        "IDLE", "Awaiting Sensor Data", 0
                    )

            except Exception as exc:
                self.connection_status_signal.emit(
                    False, f"Polling error: {exc}"
                )
                self.sensor_status_signal.emit("DISCONNECTED", "Offline", 0)
                break

            time.sleep(0.05)


class AutonomousMissionWorker(QThread):
    status_signal = Signal(str, str)
    completed_signal = Signal(bool, str)

    def __init__(self, vehicle_name="Drone1", ip="127.0.0.1", port=41451):
        super().__init__()
        self.vehicle_name = vehicle_name
        self.ip = ip
        self.port = port
        self.paused = False
        self.cancelled = False
        self.client = None

    def pause(self):
        self.paused = True
        if self.client:
            try:
                self.client.hoverAsync(vehicle_name=self.vehicle_name)
            except Exception:
                pass

    def resume(self):
        self.paused = False

    def cancel(self):
        self.cancelled = True
        self.paused = False

    def run(self):
        try:
            self.client = airsim.MultirotorClient(
                ip=self.ip, port=self.port, timeout_value=5
            )
            self.client.confirmConnection()

            # Step 1: Enable API Control & Arm
            self.status_signal.emit(
                "ARMING", f"Enabling API control & Arming {self.vehicle_name}..."
            )
            self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
            if not self.client.armDisarm(True, vehicle_name=self.vehicle_name):
                raise RuntimeError("AirSim refused arm command.")

            # Step 2: Takeoff to 3.0 meters altitude
            self.status_signal.emit(
                "TAKEOFF", "Initiating autonomous takeoff to 3.0m..."
            )
            self.client.takeoffAsync(
                timeout_sec=15, vehicle_name=self.vehicle_name
            ).join()
            time.sleep(1.0)

            # Step 3: Square Scan Pattern (8m x 8m)
            waypoints = [
                ("WAYPOINT 1/4", 1.8, 0.0, -3.0, 4.5, "Moving North (+8m)..."),
                ("WAYPOINT 2/4", 0.0, 1.8, -3.0, 4.5, "Moving East (+8m)..."),
                ("WAYPOINT 3/4", -1.8, 0.0, -3.0, 4.5, "Moving South (-8m)..."),
                ("WAYPOINT 4/4", 0.0, -1.8, -3.0, 4.5, "Moving West (-8m)..."),
            ]

            for wp_name, vx, vy, z_target, duration, msg in waypoints:
                if self.cancelled:
                    break

                while self.paused:
                    self.status_signal.emit("PAUSED", "Mission paused. Hovering in place.")
                    time.sleep(0.5)
                    if self.cancelled:
                        break

                if self.cancelled:
                    break

                self.status_signal.emit("NAVIGATING", f"{wp_name}: {msg}")
                self.client.moveByVelocityZAsync(
                    vx, vy, z_target, duration,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(False, 0),
                    vehicle_name=self.vehicle_name
                ).join()

                # Hover briefly at waypoint to capture clean LiDAR scan
                self.status_signal.emit("SCANNING", f"{wp_name}: Hovering to record LiDAR scan...")
                self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
                time.sleep(1.5)

            if not self.cancelled:
                self.status_signal.emit(
                    "HOVERING", "Mission flight pattern completed! Hovering at origin."
                )
                self.completed_signal.emit(
                    True, "Autonomous mapping mission completed successfully."
                )
            else:
                self.status_signal.emit("STOPPED", "Mission stopped by user request.")
                self.completed_signal.emit(False, "Mission stopped.")

        except Exception as exc:
            self.status_signal.emit("ERROR", f"Mission error: {exc}")
            self.completed_signal.emit(False, str(exc))


class LandingWorker(QThread):
    status_signal = Signal(str)
    completed_signal = Signal(bool, str)

    def __init__(self, vehicle_name="Drone1", ip="127.0.0.1", port=41451):
        super().__init__()
        self.vehicle_name = vehicle_name
        self.ip = ip
        self.port = port

    def run(self):
        try:
            client = airsim.MultirotorClient(
                ip=self.ip, port=self.port, timeout_value=5
            )
            client.confirmConnection()
            self.status_signal.emit(f"Initiating landing for {self.vehicle_name}...")
            client.landAsync(
                timeout_sec=30, vehicle_name=self.vehicle_name
            ).join()

            state = client.getMultirotorState(vehicle_name=self.vehicle_name)
            if state.landed_state == airsim.LandedState.Landed:
                client.armDisarm(False, vehicle_name=self.vehicle_name)
                client.enableApiControl(False, vehicle_name=self.vehicle_name)
                self.completed_signal.emit(
                    True, "Landing verified. Vehicle disarmed safely."
                )
            else:
                self.completed_signal.emit(
                    False, "Landing completed but landed_state unconfirmed."
                )
        except Exception as exc:
            self.completed_signal.emit(False, f"Landing failed: {exc}")


# --- Maximized 3D Spatial Map Window ---

class SpatialMapWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("LifeMAS | 3D Spatial LiDAR Mapping Workspace")
        self.resize(1280, 800)
        self.max_points = 300000
        self.accumulated_points = np.empty((0, 3), dtype=np.float32)

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Header bar
        header = QFrame()
        header.setObjectName("map_header")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(16, 10, 16, 10)

        title = QLabel("3D REAL-TIME SPATIAL LIDAR MAP")
        title.setObjectName("map_title")

        self.sensor_badge = QLabel("SENSOR: AWAITING DATA")
        self.sensor_badge.setObjectName("badge_neutral")

        self.pts_badge = QLabel("POINTS: 0")
        self.pts_badge.setObjectName("badge_neutral")

        self.mission_badge = QLabel("MISSION: IDLE")
        self.mission_badge.setObjectName("badge_neutral")

        self.reset_map_btn = QPushButton("Reset Map")
        self.reset_map_btn.setObjectName("btn_small")
        self.reset_map_btn.clicked.connect(self.reset_map)

        self.reset_view_btn = QPushButton("Reset View")
        self.reset_view_btn.setObjectName("btn_small")
        self.reset_view_btn.clicked.connect(self.reset_view)

        self.back_btn = QPushButton("◀ Return to Simulator View")
        self.back_btn.setObjectName("btn_primary_small")
        self.back_btn.clicked.connect(self.hide)

        header_layout.addWidget(title)
        header_layout.addStretch()
        header_layout.addWidget(self.sensor_badge)
        header_layout.addWidget(self.pts_badge)
        header_layout.addWidget(self.mission_badge)
        header_layout.addWidget(self.reset_map_btn)
        header_layout.addWidget(self.reset_view_btn)
        header_layout.addWidget(self.back_btn)
        layout.addWidget(header)

        # OpenGL 3D Viewport
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setBackgroundColor('#080a0e')
        self.gl_widget.setCameraPosition(distance=35, elevation=25, azimuth=45)

        self.grid_item = gl.GLGridItem()
        self.grid_item.setSize(x=140, y=140, z=0)
        self.grid_item.setSpacing(x=5, y=5, z=5)
        self.grid_item.setColor((0.15, 0.22, 0.3, 0.6))
        self.gl_widget.addItem(self.grid_item)

        self.axis_item = gl.GLAxisItem()
        self.axis_item.setSize(x=5, y=5, z=5)
        self.gl_widget.addItem(self.axis_item)

        self.scatter_item = gl.GLScatterPlotItem(
            pos=np.empty((0, 3), dtype=np.float32),
            size=2.5,
            pxMode=True
        )
        self.gl_widget.addItem(self.scatter_item)

        self.drone_marker = gl.GLScatterPlotItem(
            pos=np.empty((0, 3), dtype=np.float32),
            color=(0.95, 0.25, 0.25, 1.0),
            size=10.0,
            pxMode=True
        )
        self.gl_widget.addItem(self.drone_marker)

        layout.addWidget(self.gl_widget, 1)

    @Slot(object, str, int, float, float, float)
    def add_point_cloud(self, pts_new, sensor_name, scan_pts, dx, dy, dz):
        if len(pts_new) > 0:
            if len(self.accumulated_points) == 0:
                self.accumulated_points = pts_new
            else:
                self.accumulated_points = np.vstack(
                    (self.accumulated_points, pts_new)
                )

            if len(self.accumulated_points) > self.max_points:
                self.accumulated_points = self.accumulated_points[-self.max_points:]

            colors = compute_point_colors(self.accumulated_points)
            self.scatter_item.setData(
                pos=self.accumulated_points, color=colors
            )

        drone_pos = np.array([[dx, dy, dz]], dtype=np.float32)
        self.drone_marker.setData(pos=drone_pos)
        self.pts_badge.setText(f"POINTS: {len(self.accumulated_points):,}")

    @Slot(str, str, int)
    def update_sensor_status(self, status, sensor_name, scan_pts):
        if status == "ACTIVE":
            self.sensor_badge.setText(f"SENSOR: {sensor_name.upper()} ({scan_pts:,} pts/scan)")
            self.sensor_badge.setObjectName("badge_active")
        elif status == "IDLE":
            self.sensor_badge.setText(f"SENSOR: {sensor_name.upper()}")
            self.sensor_badge.setObjectName("badge_warning")
        else:
            self.sensor_badge.setText("SENSOR: OFFLINE")
            self.sensor_badge.setObjectName("badge_neutral")
        self.sensor_badge.style().unpolish(self.sensor_badge)
        self.sensor_badge.style().polish(self.sensor_badge)

    @Slot(str, str)
    def update_mission_status(self, phase, msg):
        self.mission_badge.setText(f"MISSION: {phase}")
        if phase in ("ARMING", "TAKEOFF", "NAVIGATING", "SCANNING"):
            self.mission_badge.setObjectName("badge_active")
        elif phase in ("HOVERING", "COMPLETE"):
            self.mission_badge.setObjectName("badge_info")
        else:
            self.mission_badge.setObjectName("badge_neutral")
        self.mission_badge.style().unpolish(self.mission_badge)
        self.mission_badge.style().polish(self.mission_badge)

    def reset_map(self):
        self.accumulated_points = np.empty((0, 3), dtype=np.float32)
        self.scatter_item.setData(pos=self.accumulated_points)
        self.pts_badge.setText("POINTS: 0")

    def reset_view(self):
        self.gl_widget.setCameraPosition(distance=35, elevation=25, azimuth=45)
        self.gl_widget.opts['center'] = QVector3D(0, 0, 0)


# --- Main LifeMAS Application Window ---

class LifeMAS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LifeMAS | Robotics Drone Operations Center")
        self.resize(1440, 900)
        self.setMinimumSize(1100, 700)

        self.process = None
        self.sim_hwnd = None
        self.worker = None
        self.mission_worker = None
        self.landing_worker = None
        self.original_style = None
        self.original_parent = None
        self.wait_count = 0
        self.selected_vehicle = "Drone1"
        self.api_connected = False

        # Create 3D Spatial Map Window (Hidden initially)
        self.map_window = SpatialMapWindow(self)

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(14, 12, 14, 10)
        outer.setSpacing(10)

        # Header Bar
        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 4)

        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        app_title = QLabel("LIFEMAS DRONE WORKSTATION")
        app_title.setObjectName("app_title")
        app_sub = QLabel("AIRSIMNH SIMULATOR WORKSPACE & SPATIAL MAPPING")
        app_sub.setObjectName("app_sub")
        title_box.addWidget(app_title)
        title_box.addWidget(app_sub)

        header.addLayout(title_box)
        header.addStretch()

        self.open_map_hdr_btn = QPushButton("Open 3D Map Window ↗")
        self.open_map_hdr_btn.setObjectName("btn_accent_hdr")
        self.open_map_hdr_btn.clicked.connect(self.open_map_window)

        self.sim_status_lbl = QLabel("● SIMULATOR STOPPED")
        self.sim_status_lbl.setObjectName("badge_status_neutral")

        self.api_status_lbl = QLabel("● API DISCONNECTED")
        self.api_status_lbl.setObjectName("badge_status_neutral")

        header.addWidget(self.open_map_hdr_btn)
        header.addWidget(self.sim_status_lbl)
        header.addWidget(self.api_status_lbl)
        outer.addLayout(header)

        # Central Splitter (Main AirSim Viewport in Center/Left, Sidebar on Right)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setObjectName("main_splitter")

        # MAIN WORKSPACE: Embedded AirSimNH Simulator Viewport (Priority 1)
        self.main_sim_viewport = Viewport()
        self.main_sim_viewport.on_resize = self.resize_hosted_window
        vp_layout = QVBoxLayout(self.main_sim_viewport)
        vp_layout.setContentsMargins(0, 0, 0, 0)
        
        self.placeholder = QLabel(
            "LIVE AIRSIMNH SIMULATOR WORKSPACE\n\n"
            "1. Click 'Launch AirSimNH' in the sidebar\n"
            "2. Wait for window detection\n"
            "3. Click 'Embed Viewport' to host Unreal Engine view here\n"
            "4. Click 'Connect API' to begin telemetry & mapping"
        )
        self.placeholder.setObjectName("placeholder_lbl")
        self.placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vp_layout.addWidget(self.placeholder)
        splitter.addWidget(self.main_sim_viewport)

        # RIGHT SIDEBAR: Operations & Mission Controls
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setMinimumWidth(280)
        sidebar.setMaximumWidth(340)
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(14, 14, 14, 14)
        sidebar_layout.setSpacing(10)

        # Section 1: Simulator & API Controls
        sidebar_layout.addWidget(self.section_header("1. SIMULATOR & API CONTROL"))

        self.launch_btn = QPushButton("Launch AirSimNH")
        self.launch_btn.setObjectName("btn_primary")
        self.launch_btn.clicked.connect(self.launch_simulator)

        embed_row = QHBoxLayout()
        embed_row.setSpacing(6)
        self.embed_btn = QPushButton("Embed Viewport")
        self.embed_btn.setObjectName("btn_secondary")
        self.embed_btn.setEnabled(False)
        self.embed_btn.clicked.connect(self.embed_window)

        self.restore_btn = QPushButton("Restore Window")
        self.restore_btn.setObjectName("btn_secondary")
        self.restore_btn.setEnabled(False)
        self.restore_btn.clicked.connect(self.restore_window)

        embed_row.addWidget(self.embed_btn)
        embed_row.addWidget(self.restore_btn)

        self.connect_api_btn = QPushButton("Connect API")
        self.connect_api_btn.setObjectName("btn_secondary")
        self.connect_api_btn.clicked.connect(self.toggle_api_connection)

        sidebar_layout.addWidget(self.launch_btn)
        sidebar_layout.addLayout(embed_row)
        sidebar_layout.addWidget(self.connect_api_btn)

        # Section 2: Autonomous Mapping Mission (Priority 4)
        sidebar_layout.addWidget(self.section_header("2. AUTONOMOUS MAPPING MISSION"))

        self.start_mission_btn = QPushButton("▶ Start Mapping Mission")
        self.start_mission_btn.setObjectName("btn_mission_start")
        self.start_mission_btn.clicked.connect(self.start_mapping_mission)

        mission_row = QHBoxLayout()
        mission_row.setSpacing(6)
        self.pause_mission_btn = QPushButton("Pause / Hover")
        self.pause_mission_btn.setObjectName("btn_secondary")
        self.pause_mission_btn.setEnabled(False)
        self.pause_mission_btn.clicked.connect(self.pause_mapping_mission)

        self.land_btn = QPushButton("Land & Disarm")
        self.land_btn.setObjectName("btn_secondary")
        self.land_btn.setEnabled(False)
        self.land_btn.clicked.connect(self.land_drone)

        mission_row.addWidget(self.pause_mission_btn)
        mission_row.addWidget(self.land_btn)

        self.mission_status_lbl = QLabel("MISSION STATUS: IDLE")
        self.mission_status_lbl.setObjectName("badge_status_neutral")
        self.mission_status_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

        sidebar_layout.addWidget(self.start_mission_btn)
        sidebar_layout.addLayout(mission_row)
        sidebar_layout.addWidget(self.mission_status_lbl)

        # Section 3: 3D Spatial Map Window Launcher (Priority 1)
        sidebar_layout.addWidget(self.section_header("3. 3D SPATIAL MAPPING VISUALIZATION"))

        self.open_map_card_btn = QPushButton("OPEN 3D MAP WINDOW ↗")
        self.open_map_card_btn.setObjectName("btn_map_card")
        self.open_map_card_btn.clicked.connect(self.open_map_window)

        map_stats_row = QHBoxLayout()
        self.sidebar_pts_lbl = QLabel("Map Points: 0")
        self.sidebar_sensor_lbl = QLabel("Sensor: Waiting")
        self.sidebar_pts_lbl.setObjectName("telem_val")
        self.sidebar_sensor_lbl.setObjectName("telem_val")
        map_stats_row.addWidget(self.sidebar_pts_lbl)
        map_stats_row.addStretch()
        map_stats_row.addWidget(self.sidebar_sensor_lbl)

        sidebar_layout.addWidget(self.open_map_card_btn)
        sidebar_layout.addLayout(map_stats_row)

        # Section 4: Telemetry (Priority 2)
        sidebar_layout.addWidget(self.section_header("4. VEHICLE TELEMETRY"))

        telem_grid = QGridLayout()
        telem_grid.setVerticalSpacing(5)
        telem_grid.setHorizontalSpacing(10)

        self.telemetry = {}
        items = [
            ("Active Vehicle", "Drone1"),
            ("Position X/Y", "0.0, 0.0"),
            ("Altitude", "0.00 m"),
            ("Velocity", "0.0, 0.0, 0.0"),
            ("Ready / Arm", "Not Checked"),
            ("Landed State", "Unknown")
        ]
        for row, (name, val_init) in enumerate(items):
            lbl_name = QLabel(name)
            lbl_name.setObjectName("telem_name")
            lbl_val = QLabel(val_init)
            lbl_val.setObjectName("telem_val")
            telem_grid.addWidget(lbl_name, row, 0)
            telem_grid.addWidget(lbl_val, row, 1)
            self.telemetry[name] = lbl_val

        sidebar_layout.addLayout(telem_grid)
        sidebar_layout.addStretch()

        splitter.addWidget(sidebar)
        splitter.setStretchFactor(0, 8)
        splitter.setStretchFactor(1, 2)
        outer.addWidget(splitter, 1)

        # Status Bar
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.status.showMessage("Workstation Ready · Click Launch AirSimNH to begin.")

        self.apply_styles()

        # Window Poll Timer
        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(1000)
        self.poll_timer.timeout.connect(self.check_simulator_window)

    @staticmethod
    def section_header(text):
        lbl = QLabel(text)
        lbl.setObjectName("section_hdr")
        return lbl

    def apply_styles(self):
        self.setStyleSheet("""
            QWidget#root {
                background-color: #0b0d11;
                color: #e2e8f0;
                font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
                font-size: 12px;
            }
            QLabel#app_title {
                color: #f8fafc;
                font-size: 16px;
                font-weight: 800;
                letter-spacing: 1.5px;
            }
            QLabel#app_sub {
                color: #64748b;
                font-size: 10px;
                font-weight: 600;
                letter-spacing: 0.5px;
            }
            QLabel#badge_status_neutral {
                background: #181d27; color: #64748b;
                padding: 5px 10px; border-radius: 4px; font-weight: 600; font-size: 11px;
            }
            QLabel#badge_status_active {
                background: #064e3b; color: #34d399;
                padding: 5px 10px; border-radius: 4px; font-weight: 600; font-size: 11px;
            }
            QLabel#section_hdr {
                color: #94a3b8; font-size: 10px; font-weight: 700;
                letter-spacing: 1px; margin-top: 6px; margin-bottom: 2px;
            }
            QFrame#sidebar {
                background: #12151c; border: 1px solid #1e2430; border-radius: 6px;
            }
            QFrame#viewport {
                background: #05070a; border: 1px solid #1e2430; border-radius: 6px;
            }
            QLabel#placeholder_lbl {
                color: #64748b; font-size: 13px; font-weight: 500; line-height: 1.6;
            }
            QLabel#telem_name { color: #64748b; font-size: 11px; }
            QLabel#telem_val { color: #cbd5e1; font-weight: 600; font-size: 11px; }

            QPushButton {
                padding: 7px 12px; border-radius: 4px; font-weight: 600; font-size: 11px;
            }
            QPushButton#btn_primary {
                background: #2563eb; color: #ffffff; border: none;
            }
            QPushButton#btn_primary:hover { background: #3b82f6; }
            QPushButton#btn_primary_small {
                background: #2563eb; color: #ffffff; border: none; padding: 4px 10px;
            }
            QPushButton#btn_primary_small:hover { background: #3b82f6; }
            QPushButton#btn_secondary {
                background: #1e293b; color: #e2e8f0; border: 1px solid #334155;
            }
            QPushButton#btn_secondary:hover { background: #334155; }
            QPushButton#btn_accent_hdr {
                background: #059669; color: #ffffff; border: none; font-weight: 700;
            }
            QPushButton#btn_accent_hdr:hover { background: #10b981; }
            QPushButton#btn_mission_start {
                background: #0d9488; color: #ffffff; border: none; font-size: 12px; font-weight: 700; padding: 9px;
            }
            QPushButton#btn_mission_start:hover { background: #14b8a6; }
            QPushButton#btn_map_card {
                background: #1e1b4b; color: #a5b4fc; border: 1px solid #3730a3;
                font-weight: 700; font-size: 12px; padding: 10px;
            }
            QPushButton#btn_map_card:hover { background: #312e81; color: #ffffff; }

            QPushButton:disabled {
                color: #475569; background: #141a24; border-color: #1e293b;
            }
            QStatusBar {
                background: #0b0d11; color: #64748b; border-top: 1px solid #1e2430;
            }

            QFrame#map_header {
                background: #12151c; border-bottom: 1px solid #1e2430;
            }
            QLabel#map_title {
                color: #cbd5e1; font-weight: 800; font-size: 12px; letter-spacing: 1px;
            }
            QLabel#badge_neutral {
                background: #1e2430; color: #94a3b8; padding: 3px 8px; border-radius: 4px; font-size: 10px; font-weight: 600;
            }
            QLabel#badge_active {
                background: #064e3b; color: #34d399; padding: 3px 8px; border-radius: 4px; font-size: 10px; font-weight: 600;
            }
            QLabel#badge_warning {
                background: #78350f; color: #fbbf24; padding: 3px 8px; border-radius: 4px; font-size: 10px; font-weight: 600;
            }
            QLabel#badge_info {
                background: #1e3a8a; color: #93c5fd; padding: 3px 8px; border-radius: 4px; font-size: 10px; font-weight: 600;
            }
            QPushButton#btn_small {
                background: #1e293b; color: #cbd5e1; border: 1px solid #334155; padding: 3px 10px; border-radius: 4px; font-size: 11px; font-weight: 600;
            }
            QPushButton#btn_small:hover { background: #334155; color: #ffffff; }
        """)

    def open_map_window(self):
        self.map_window.showMaximized()
        self.map_window.raise_()
        self.map_window.activateWindow()

    # --- API Connection Methods ---

    def toggle_api_connection(self):
        if self.worker and self.worker.isRunning():
            self.disconnect_api()
        else:
            self.connect_api()

    def connect_api(self):
        self.connect_api_btn.setEnabled(False)
        self.connect_api_btn.setText("Connecting...")

        self.worker = LidarTelemetryWorker()
        self.worker.set_vehicle(self.selected_vehicle)
        self.worker.lidar_data_signal.connect(self.map_window.add_point_cloud)
        self.worker.lidar_data_signal.connect(self.update_sidebar_map_stats)
        self.worker.sensor_status_signal.connect(self.map_window.update_sensor_status)
        self.worker.telemetry_signal.connect(self.update_telemetry_ui)
        self.worker.connection_status_signal.connect(self.handle_connection_status)
        self.worker.start()

    def disconnect_api(self):
        if self.worker:
            self.worker.stop()
            self.worker = None

        self.api_connected = False
        self.api_status_lbl.setText("● API DISCONNECTED")
        self.api_status_lbl.setObjectName("badge_status_neutral")
        self.api_status_lbl.style().unpolish(self.api_status_lbl)
        self.api_status_lbl.style().polish(self.api_status_lbl)

        self.connect_api_btn.setText("Connect API")
        self.connect_api_btn.setEnabled(True)
        self.land_btn.setEnabled(False)
        self.status.showMessage("AirSim API disconnected.")

    @Slot(bool, str)
    def handle_connection_status(self, connected, msg):
        self.connect_api_btn.setEnabled(True)
        self.api_connected = connected
        if connected:
            self.connect_api_btn.setText("Disconnect API")
            self.api_status_lbl.setText("● AIRSIM API CONNECTED")
            self.api_status_lbl.setObjectName("badge_status_active")
            self.land_btn.setEnabled(True)
            self.status.showMessage(f"Connected: {msg}")
        else:
            self.connect_api_btn.setText("Connect API")
            self.api_status_lbl.setText("● API DISCONNECTED")
            self.api_status_lbl.setObjectName("badge_status_neutral")
            self.land_btn.setEnabled(False)
            self.status.showMessage(f"Connection state: {msg}")

        self.api_status_lbl.style().unpolish(self.api_status_lbl)
        self.api_status_lbl.style().polish(self.api_status_lbl)

    @Slot(object, str, int, float, float, float)
    def update_sidebar_map_stats(self, pts_new, sensor_name, scan_pts, dx, dy, dz):
        self.sidebar_pts_lbl.setText(f"Map Points: {len(self.map_window.accumulated_points):,}")
        self.sidebar_sensor_lbl.setText(f"Sensor: {sensor_name}")

    @Slot(dict)
    def update_telemetry_ui(self, t):
        vehicles_str = ", ".join(t["active_vehicles"]) if t["active_vehicles"] else "None"
        self.telemetry["Active Vehicle"].setText(vehicles_str)
        self.telemetry["Position X/Y"].setText(f"{t['pos_x']:.1f}, {t['pos_y']:.1f}")
        self.telemetry["Altitude"].setText(f"{t['altitude']:.2f} m")
        self.telemetry["Velocity"].setText(f"{t['vel_x']:.1f}, {t['vel_y']:.1f}, {t['vel_z']:.1f}")
        self.telemetry["Ready / Arm"].setText(f"R:{t['ready']} | Arm:{t['can_arm']}")
        self.telemetry["Landed State"].setText(t["landed_state"].replace("LandedState.", ""))

    # --- Mission Control Methods (Priority 4) ---

    def start_mapping_mission(self):
        if not self.api_connected:
            QMessageBox.warning(
                self, "API Disconnected",
                "Please click 'Connect API' before starting the mapping mission."
            )
            return

        if self.mission_worker and self.mission_worker.isRunning():
            if self.mission_worker.paused:
                self.mission_worker.resume()
                self.start_mission_btn.setText("▶ Resume Mapping Mission")
                self.pause_mission_btn.setEnabled(True)
                self.status.showMessage("Resuming mapping mission...")
            return

        self.start_mission_btn.setEnabled(False)
        self.pause_mission_btn.setEnabled(True)
        self.land_btn.setEnabled(True)

        self.mission_worker = AutonomousMissionWorker(self.selected_vehicle)
        self.mission_worker.status_signal.connect(self.handle_mission_status)
        self.mission_worker.status_signal.connect(self.map_window.update_mission_status)
        self.mission_worker.completed_signal.connect(self.finish_mapping_mission)
        self.mission_worker.start()

    def pause_mapping_mission(self):
        if self.mission_worker and self.mission_worker.isRunning():
            if not self.mission_worker.paused:
                self.mission_worker.pause()
                self.pause_mission_btn.setText("Resume Mission")
                self.start_mission_btn.setEnabled(True)
                self.start_mission_btn.setText("▶ Resume Mapping Mission")
                self.status.showMessage("Mapping mission paused. Hovering in place.")
            else:
                self.mission_worker.resume()
                self.pause_mission_btn.setText("Pause / Hover")
                self.start_mission_btn.setEnabled(False)
                self.status.showMessage("Resuming mapping mission...")

    def land_drone(self):
        if not self.api_connected:
            QMessageBox.warning(
                self, "API Disconnected",
                "Connect API before requesting landing."
            )
            return

        self.status.showMessage("Initiating landing procedure...")
        if self.mission_worker and self.mission_worker.isRunning():
            self.mission_worker.cancel()

        self.start_mission_btn.setEnabled(False)
        self.pause_mission_btn.setEnabled(False)
        self.land_btn.setEnabled(False)

        self.landing_worker = LandingWorker(self.selected_vehicle)
        self.landing_worker.status_signal.connect(self.status.showMessage)
        self.landing_worker.completed_signal.connect(self.finish_landing)
        self.landing_worker.start()

    @Slot(str, str)
    def handle_mission_status(self, phase, msg):
        self.mission_status_lbl.setText(f"MISSION: {phase}")
        self.status.showMessage(f"[{phase}] {msg}")

    @Slot(bool, str)
    def finish_mapping_mission(self, success, msg):
        self.start_mission_btn.setEnabled(True)
        self.start_mission_btn.setText("▶ Start Mapping Mission")
        self.pause_mission_btn.setEnabled(False)
        self.pause_mission_btn.setText("Pause / Hover")
        self.status.showMessage(f"Mission: {msg}")

    @Slot(bool, str)
    def finish_landing(self, success, msg):
        self.start_mission_btn.setEnabled(True)
        self.start_mission_btn.setText("▶ Start Mapping Mission")
        self.pause_mission_btn.setEnabled(False)
        self.land_btn.setEnabled(True)
        self.mission_status_lbl.setText("MISSION: LANDED & DISARMED")
        self.status.showMessage(msg)

    # --- Simulator Launch & Window Embedding ---

    def launch_simulator(self):
        if not os.path.isfile(SIMULATOR):
            QMessageBox.critical(
                self, "Simulator Missing",
                f"AirSimNH.exe not found at:\n{SIMULATOR}"
            )
            return

        if self.process and self.process.poll() is None:
            self.status.showMessage("AirSimNH simulator process is already running.")
            return

        try:
            self.process = subprocess.Popen(
                [SIMULATOR],
                cwd=os.path.dirname(SIMULATOR)
            )
        except Exception as exc:
            QMessageBox.critical(self, "Launch Error", str(exc))
            return

        self.wait_count = 0
        self.launch_btn.setEnabled(False)
        self.sim_status_lbl.setText("● STARTING SIMULATOR...")
        self.status.showMessage("Launching AirSimNH. Waiting for window handle...")
        self.poll_timer.start()

    def check_simulator_window(self):
        self.wait_count += 1
        if self.process and self.process.poll() is not None:
            self.poll_timer.stop()
            self.launch_btn.setEnabled(True)
            self.sim_status_lbl.setText("● SIMULATOR EXITED")
            self.status.showMessage("AirSimNH process exited.")
            return

        if self.process:
            hwnd = find_window_for_pid(self.process.pid)
            if hwnd:
                self.sim_hwnd = hwnd
                self.poll_timer.stop()
                self.embed_btn.setEnabled(True)
                self.sim_status_lbl.setText("● SIMULATOR RUNNING")
                self.sim_status_lbl.setObjectName("badge_status_active")
                self.sim_status_lbl.style().unpolish(self.sim_status_lbl)
                self.sim_status_lbl.style().polish(self.sim_status_lbl)
                self.status.showMessage("AirSimNH window detected. Ready to embed into workspace.")
                return

        if self.wait_count >= 120:
            self.poll_timer.stop()
            self.launch_btn.setEnabled(True)
            self.status.showMessage("AirSimNH window detection timed out.")

    def embed_window(self):
        if not self.sim_hwnd or not user32.IsWindow(self.sim_hwnd):
            QMessageBox.warning(
                self, "Window Error", "AirSimNH window unavailable."
            )
            return

        hwnd = self.sim_hwnd
        parent = int(self.main_sim_viewport.winId())
        self.original_style = get_style(hwnd, GWL_STYLE)
        self.original_parent = user32.SetParent(hwnd, parent)

        child_style = (
            self.original_style | WS_CHILD | 0x10000000
        ) & ~(
            WS_POPUP | WS_CAPTION | WS_THICKFRAME |
            WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU
        )
        set_style(hwnd, GWL_STYLE, child_style)
        user32.ShowWindow(hwnd, SW_RESTORE)
        self.resize_hosted_window()

        self.main_sim_viewport.hosted_hwnd = hwnd
        self.placeholder.hide()
        self.restore_btn.setEnabled(True)
        self.embed_btn.setEnabled(False)
        self.status.showMessage("AirSimNH window embedded into primary workspace.")

    def resize_hosted_window(self):
        if self.sim_hwnd and user32.IsWindow(self.sim_hwnd):
            user32.SetWindowPos(
                self.sim_hwnd, 0, 0, 0,
                self.main_sim_viewport.width(), self.main_sim_viewport.height(),
                SWP_NOZORDER | SWP_FRAMECHANGED | SWP_SHOWWINDOW
            )

    def restore_window(self):
        if self.sim_hwnd and user32.IsWindow(self.sim_hwnd):
            user32.SetParent(self.sim_hwnd, None)
            if self.original_style is not None:
                set_style(self.sim_hwnd, GWL_STYLE, self.original_style)
            user32.SetWindowPos(
                self.sim_hwnd, 0, 100, 100, 1280, 720,
                SWP_NOZORDER | SWP_FRAMECHANGED | SWP_SHOWWINDOW
            )
            user32.ShowWindow(self.sim_hwnd, SW_RESTORE)

        self.main_sim_viewport.hosted_hwnd = None
        self.placeholder.show()
        self.restore_btn.setEnabled(False)
        self.embed_btn.setEnabled(bool(self.sim_hwnd))
        self.status.showMessage("AirSimNH window restored to standalone window.")

    def closeEvent(self, event):
        if self.worker:
            self.worker.stop()
        if self.main_sim_viewport.hosted_hwnd:
            self.restore_window()
        self.map_window.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = LifeMAS()
    window.show()
    sys.exit(app.exec())
