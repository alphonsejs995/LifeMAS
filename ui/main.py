import ctypes
import os
import subprocess
import sys
import time
import math
import threading
import numpy as np
import airsim
from ctypes import wintypes

from PySide6.QtCore import Qt, QTimer, QThread, Signal, Slot, QPointF, QRectF
from PySide6.QtGui import QVector3D, QPainter, QColor, QPen, QBrush
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QStatusBar, QMessageBox,
    QSplitter, QStackedWidget, QProgressBar, QFileDialog
)
import pyqtgraph.opengl as gl

PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIMULATOR = os.path.join(
    PROJECT, "AirSimNH", "AirSimNH",
    "WindowsNoEditor", "AirSimNH.exe"
)

# Import Mapping Engines
sys.path.insert(0, PROJECT)
from mapping.persistent_map import PersistentMapManager
from mapping.survey_engine import (
    SurveyConfig,
    generate_survey_waypoints,
    SurveyDataset,
    SurveyArtifact,
    ReconstructionEngine,
    export_survey_point_cloud
)

MAP_SAVE_FILE = os.path.join(PROJECT, "data", "maps", "airsim_map.npz")
FINAL_SURVEY_LATEST = os.path.join(PROJECT, "data", "maps", "final_survey_latest.npz")

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


# --- Spatial Math Transformations ---

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


def quaternion_to_yaw(q):
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_sensor_to_world_vis(pts_sensor, sensor_pose, vehicle_pose=None):
    if len(pts_sensor) == 0:
        return np.empty((0, 3), dtype=np.float32)

    pts = np.asarray(pts_sensor, dtype=np.float32)
    Rs = quaternion_to_rotation_matrix(sensor_pose.orientation)
    Ts = np.array([
        sensor_pose.position.x_val,
        sensor_pose.position.y_val,
        sensor_pose.position.z_val
    ], dtype=np.float32)

    # In AirSim, lidar_data.pose is already the sensor's world NED pose (params.relative_pose + kinematics.pose).
    # If vehicle_pose is provided and differs substantially from sensor_pose (e.g. sensor_pose is local mounting offset),
    # apply two-stage transform. Otherwise, sensor_pose is already the world pose.
    is_local_offset = False
    if vehicle_pose is not None:
        pos_v = getattr(vehicle_pose, 'position', None)
        if pos_v is not None:
            Tv = np.array([pos_v.x_val, pos_v.y_val, pos_v.z_val], dtype=np.float32)
            if not np.allclose(Ts, Tv, atol=0.05):
                is_local_offset = True

    if is_local_offset:
        pts_drone = pts @ Rs.T + Ts
        ori_v = getattr(vehicle_pose, 'orientation', None)
        Rv = quaternion_to_rotation_matrix(ori_v) if ori_v is not None else np.eye(3, dtype=np.float32)
        pts_world_ned = pts_drone @ Rv.T + Tv
    else:
        pts_world_ned = pts @ Rs.T + Ts

    # Convert AirSim NED (X North, Y East, Z Down) to Visualization (X North, Y East, Z Up)
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


# --- High-Performance 2D Occupancy Grid Viewport Widget ---

class OccupancyGridWidget(QWidget):
    def __init__(self, map_manager, parent=None):
        super().__init__(parent)
        self.map_manager = map_manager
        self.scale = 22.0  # Pixels per meter
        self.offset_x = 0.0  # World X center offset
        self.offset_y = 0.0  # World Y center offset
        
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_yaw = 0.0  # Radians
        
        self.dragging = False
        self.last_mouse_pos = None

        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def set_drone_pose(self, x, y, yaw_rad):
        self.drone_x = float(x)
        self.drone_y = float(y)
        self.drone_yaw = float(yaw_rad)

    def fit_to_map(self):
        if self.map_manager.has_bounds:
            min_x, max_x = self.map_manager.min_x, self.map_manager.max_x
            min_y, max_y = self.map_manager.min_y, self.map_manager.max_y
            
            span_x = max(max_x - min_x, 6.0)
            span_y = max(max_y - min_y, 6.0)
            
            self.offset_x = (min_x + max_x) / 2.0
            self.offset_y = (min_y + max_y) / 2.0
            
            w_pixels = max(self.width(), 100)
            h_pixels = max(self.height(), 100)
            
            scale_x = (w_pixels * 0.75) / span_x
            scale_y = (h_pixels * 0.75) / span_y
            self.scale = max(4.0, min(scale_x, scale_y, 120.0))
        else:
            self.offset_x = self.drone_x
            self.offset_y = self.drone_y
            self.scale = 22.0
        self.update()

    def reset_view(self):
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.scale = 22.0
        self.update()

    def world_to_screen(self, wx, wy):
        cx = self.width() / 2.0
        cy = self.height() / 2.0
        sx = cx + (wx - self.offset_x) * self.scale
        sy = cy - (wy - self.offset_y) * self.scale
        return sx, sy

    def mousePressEvent(self, event):
        if event.button() in (Qt.MouseButton.LeftButton, Qt.MouseButton.MiddleButton):
            self.dragging = True
            self.last_mouse_pos = event.position()

    def mouseMoveEvent(self, event):
        if self.dragging and self.last_mouse_pos is not None:
            delta = event.position() - self.last_mouse_pos
            self.last_mouse_pos = event.position()
            self.offset_x -= delta.x() / self.scale
            self.offset_y += delta.y() / self.scale
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() in (Qt.MouseButton.LeftButton, Qt.MouseButton.MiddleButton):
            self.dragging = False
            self.last_mouse_pos = None

    def wheelEvent(self, event):
        angle = event.angleDelta().y()
        factor = 1.15 if angle > 0 else 0.85
        self.scale = max(2.0, min(self.scale * factor, 300.0))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # 1. Neutral Unknown Dark Background (#080a0e)
        painter.fillRect(self.rect(), QColor("#080a0e"))

        # 2. Fast Vectorized QImage Raster Layer (Sub-millisecond Blit!)
        qimg, w_min_x, w_max_x, w_min_y, w_max_y, cell_size = self.map_manager.get_occupancy_qimage()
        if qimg is not None:
            sx0, sy1 = self.world_to_screen(w_min_x, w_min_y)
            sx1, sy0 = self.world_to_screen(w_max_x, w_max_y)
            
            screen_x = min(sx0, sx1)
            screen_y = min(sy0, sy1)
            screen_w = abs(sx1 - sx0)
            screen_h = abs(sy1 - sy0)
            
            target_rect = QRectF(screen_x, screen_y, screen_w, screen_h)
            painter.drawImage(target_rect, qimg)

        # 3. Draw World Axes
        ox, oy = self.world_to_screen(0, 0)
        
        pen_axis_x = QPen(QColor(239, 68, 68, 160), 1.5) # +X Red (North)
        pen_axis_y = QPen(QColor(34, 197, 94, 160), 1.5) # +Y Green (East)
        
        painter.setPen(pen_axis_x)
        x_end_x, x_end_y = self.world_to_screen(3.0, 0)
        painter.drawLine(QPointF(ox, oy), QPointF(x_end_x, x_end_y))
        
        painter.setPen(pen_axis_y)
        y_end_x, y_end_y = self.world_to_screen(0, 3.0)
        painter.drawLine(QPointF(ox, oy), QPointF(y_end_x, y_end_y))

        # 4. Render Drone Position Marker & Heading Direction Vector
        dx_screen, dy_screen = self.world_to_screen(self.drone_x, self.drone_y)
        
        # Drone Red Dot
        painter.setPen(QPen(QColor("#f8fafc"), 1.5))
        painter.setBrush(QBrush(QColor("#ef4444")))
        painter.drawEllipse(QPointF(dx_screen, dy_screen), 6.5, 6.5)

        # Drone Heading Vector Line (Yaw angle)
        head_len = 18.0
        hx = dx_screen + head_len * math.cos(self.drone_yaw)
        hy = dy_screen - head_len * math.sin(self.drone_yaw)
        
        painter.setPen(QPen(QColor("#facc15"), 2.2))
        painter.drawLine(QPointF(dx_screen, dy_screen), QPointF(hx, hy))
        painter.end()


# --- Workers ---

class LidarTelemetryWorker(QThread):
    scan_processed_signal = Signal(int, int, float, float, float, float) # (scan_pts, total_pts, dx, dy, dz, yaw)
    telemetry_signal = Signal(dict)
    sensor_status_signal = Signal(str, str, int)
    connection_status_signal = Signal(bool, str)

    def __init__(self, map_manager, ip="127.0.0.1", port=41451):
        super().__init__()
        self.map_manager = map_manager
        self.ip = ip
        self.port = port
        self.selected_vehicle = "Drone1"
        self.running = False
        self.client = None
        self.survey_dataset = None

    def set_survey_dataset(self, dataset):
        self.survey_dataset = dataset

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
                ori = state.kinematics_estimated.orientation
                
                dx_vis, dy_vis, dz_vis = pos.x_val, pos.y_val, -pos.z_val
                yaw_rad = quaternion_to_yaw(ori)

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
                        # INTEGRATE MAP IN BACKGROUND THREAD (Zero GUI thread blocking!)
                        self.map_manager.add_lidar_scan(pts_vis, dx_vis, dy_vis, dz_vis)
                        
                        # RECORD TO RECOVERABLE SURVEY DATASET IF ACTIVE
                        if self.survey_dataset is not None:
                            self.survey_dataset.add_scan(
                                timestamp=time.time(),
                                vehicle_pos=(pos.x_val, pos.y_val, pos.z_val),
                                vehicle_ori=(ori.w_val, ori.x_val, ori.y_val, ori.z_val),
                                sensor_pos=(lidar_data.pose.position.x_val, lidar_data.pose.position.y_val, lidar_data.pose.position.z_val),
                                sensor_ori=(lidar_data.pose.orientation.w_val, lidar_data.pose.orientation.x_val, lidar_data.pose.orientation.y_val, lidar_data.pose.orientation.z_val),
                                points_sensor=pts_valid
                            )

                        total_pts = len(self.map_manager.voxel_points)
                        self.scan_processed_signal.emit(
                            len(pts_vis), total_pts, dx_vis, dy_vis, dz_vis, yaw_rad
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
                                self.map_manager.add_lidar_scan(pts_vis, dx_vis, dy_vis, dz_vis)
                                total_pts = len(self.map_manager.voxel_points)
                                self.scan_processed_signal.emit(
                                    len(pts_vis), total_pts, dx_vis, dy_vis, dz_vis, yaw_rad
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
            connected = False
            for attempt in range(8):
                try:
                    self.client = airsim.MultirotorClient(
                        ip=self.ip, port=self.port, timeout_value=3
                    )
                    self.client.confirmConnection()
                    connected = True
                    break
                except Exception:
                    self.status_signal.emit(
                        "CONNECTING", f"Waiting for AirSim API connection (attempt {attempt+1}/8)..."
                    )
                    time.sleep(1.0)

            if not connected:
                raise RuntimeError(
                    "Could not connect to AirSim API (127.0.0.1:41451). Make sure AirSimNH is running."
                )

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


class SurveyMissionWorker(QThread):
    status_signal = Signal(str, str, int)       # phase, message, progress_pct (0-100)
    completed_signal = Signal(bool, str)        # success, message

    def __init__(self, vehicle_name="Drone1", survey_config=None, ip="127.0.0.1", port=41451):
        super().__init__()
        self.vehicle_name = vehicle_name
        self.config = survey_config or SurveyConfig()
        self.ip = ip
        self.port = port
        self.client = None
        self.cancelled = False
        self.paused = False

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
            connected = False
            for attempt in range(8):
                try:
                    self.client = airsim.MultirotorClient(ip=self.ip, port=self.port, timeout_value=3)
                    self.client.confirmConnection()
                    connected = True
                    break
                except Exception:
                    self.status_signal.emit("CONNECTING", f"Connecting to AirSim API (attempt {attempt+1}/8)...", 2)
                    time.sleep(1.0)

            if not connected:
                raise RuntimeError("Could not connect to AirSim API (127.0.0.1:41451). Make sure AirSimNH is running.")

            # Step 1: Arm & Enable API Control
            self.status_signal.emit("ARMING", f"Enabling API control & Arming {self.vehicle_name}...", 5)
            self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
            if not self.client.armDisarm(True, vehicle_name=self.vehicle_name):
                raise RuntimeError("AirSim refused arm command.")

            # Step 2: Takeoff to Survey Altitude
            alt_m = self.config.altitude
            self.status_signal.emit("TAKEOFF", f"Climbing to survey altitude {alt_m:.1f}m...", 10)
            self.client.takeoffAsync(timeout_sec=15, vehicle_name=self.vehicle_name).join()
            time.sleep(1.0)

            # Generate Boustrophedon Waypoints
            waypoints = generate_survey_waypoints(self.config)
            flight_wps = [w for w in waypoints if w["type"] in ("TRANSITION", "PASS", "TURN", "RETURN")]
            total_flight_wps = len(flight_wps)

            for idx, wp in enumerate(flight_wps):
                if self.cancelled:
                    break

                while self.paused:
                    self.status_signal.emit(
                        "PAUSED",
                        f"Survey paused during {wp['name']}. Hovering.",
                        int(15 + (idx / total_flight_wps) * 70)
                    )
                    try:
                        self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
                    except Exception:
                        pass
                    time.sleep(0.5)
                    if self.cancelled:
                        break

                if self.cancelled:
                    break

                progress = 15 + int((idx / total_flight_wps) * 70) # 15% -> 85%
                self.status_signal.emit("SURVEYING", f"{wp['name']}: {wp['description']}", progress)

                dist = math.hypot(wp["x"], wp["y"])
                timeout = max(10.0, (dist / max(0.5, wp["speed"])) * 2.5 + 8.0)
                self.client.moveToPositionAsync(
                    wp["x"], wp["y"], wp["z"], wp["speed"],
                    timeout_sec=timeout,
                    drivetrain=airsim.DrivetrainType.ForwardOnly,
                    yaw_mode=airsim.YawMode(False, 0),
                    vehicle_name=self.vehicle_name
                ).join()

            # Step 3: Return to Launch Origin
            if not self.cancelled:
                self.status_signal.emit("RETURNING", "Survey passes complete. Returning home to origin...", 88)
                self.client.moveToPositionAsync(
                    0.0, 0.0, -2.5, 1.2, timeout_sec=15,
                    vehicle_name=self.vehicle_name
                ).join()
            else:
                self.status_signal.emit("ABORTING", "Survey aborted. Returning for safe landing...", 88)
                try:
                    self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
                    time.sleep(0.5)
                except Exception:
                    pass

            # Step 4: Safe Landing Execution
            self.status_signal.emit("LANDING", f"Initiating landing for {self.vehicle_name}...", 92)
            self.client.landAsync(timeout_sec=30, vehicle_name=self.vehicle_name).join()

            # Step 5: Confirm Landing
            self.status_signal.emit("CONFIRMING", "Confirming ground landing state...", 94)
            confirmed = False
            for _ in range(25):
                st = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
                landed_val = str(st.landed_state)
                alt = abs(st.kinematics_estimated.position.z_val)
                vz = abs(st.kinematics_estimated.linear_velocity.z_val)
                if "Landed" in landed_val or (alt < 0.35 and vz < 0.2):
                    confirmed = True
                    break
                time.sleep(0.5)

            # Disarm and release control
            try:
                self.client.armDisarm(False, vehicle_name=self.vehicle_name)
                self.client.enableApiControl(False, vehicle_name=self.vehicle_name)
            except Exception:
                pass

            if not self.cancelled and confirmed:
                self.status_signal.emit("LANDED", "Drone safely landed and disarmed.", 96)
                self.completed_signal.emit(True, "Survey flight and landing completed successfully.")
            elif self.cancelled:
                self.status_signal.emit("STOPPED", "Survey aborted. Drone landed safely.", 96)
                self.completed_signal.emit(False, "Survey flight was aborted by user.")
            else:
                self.status_signal.emit("WARNING", "Landing timeout. Checking sensor data...", 96)
                self.completed_signal.emit(True, "Survey flight completed with landing timeout.")

        except Exception as exc:
            self.status_signal.emit("ERROR", f"Survey flight error: {exc}", 0)
            self.completed_signal.emit(False, str(exc))


class MapReconstructionWorker(QThread):
    progress_signal = Signal(int, str)          # progress_percent, message
    finished_signal = Signal(bool, object, str) # success, artifact, error_msg

    def __init__(self, dataset, output_dir):
        super().__init__()
        self.dataset = dataset
        self.output_dir = output_dir

    def run(self):
        try:
            artifact = ReconstructionEngine.process_survey(
                self.dataset,
                self.output_dir,
                progress_callback=lambda pct, msg: self.progress_signal.emit(pct, msg)
            )
            self.finished_signal.emit(True, artifact, "")
        except Exception as exc:
            self.finished_signal.emit(False, None, str(exc))


# --- Maximized Persistent Spatial Map Window ---

class SpatialMapWindow(QMainWindow):
    save_status_signal = Signal(bool, str)
    load_status_signal = Signal(bool, str)

    def __init__(self, map_manager, parent=None):
        super().__init__(parent)
        self.map_manager = map_manager
        self.setWindowTitle("LifeMAS | Persistent World-Frame Mapping Workspace")
        self.resize(1280, 800)

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Header bar
        header = QFrame()
        header.setObjectName("map_header")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(14, 8, 14, 8)

        title = QLabel("PERSISTENT SPATIAL MAP")
        title.setObjectName("map_title")

        # View Mode Toggle Buttons
        self.btn_2d_mode = QPushButton("2D Vacuum Grid Mode")
        self.btn_2d_mode.setObjectName("btn_mode_active")
        self.btn_2d_mode.clicked.connect(self.show_2d_mode)

        self.btn_3d_mode = QPushButton("3D Point Cloud View")
        self.btn_3d_mode.setObjectName("btn_mode_inactive")
        self.btn_3d_mode.clicked.connect(self.show_3d_mode)

        self.sensor_badge = QLabel("SENSOR: AWAITING DATA")
        self.sensor_badge.setObjectName("badge_neutral")

        self.stats_badge = QLabel("EXPLORED: 0.0 m² | 3D PTS: 0")
        self.stats_badge.setObjectName("badge_neutral")

        self.mission_badge = QLabel("MISSION: IDLE")
        self.mission_badge.setObjectName("badge_neutral")

        self.save_map_btn = QPushButton("Save Map")
        self.save_map_btn.setObjectName("btn_small")
        self.save_map_btn.clicked.connect(self.save_map_action)

        self.load_map_btn = QPushButton("Load Map")
        self.load_map_btn.setObjectName("btn_small")
        self.load_map_btn.clicked.connect(self.load_map_action)

        self.fit_map_btn = QPushButton("Fit to Map")
        self.fit_map_btn.setObjectName("btn_small")
        self.fit_map_btn.clicked.connect(self.fit_to_map)

        self.reset_map_btn = QPushButton("Reset Map")
        self.reset_map_btn.setObjectName("btn_small")
        self.reset_map_btn.clicked.connect(self.reset_map_action)

        self.back_btn = QPushButton("◀ Return to Simulator View")
        self.back_btn.setObjectName("btn_primary_small")
        self.back_btn.clicked.connect(self.hide)

        header_layout.addWidget(title)
        header_layout.addWidget(self.btn_2d_mode)
        header_layout.addWidget(self.btn_3d_mode)
        header_layout.addStretch()
        header_layout.addWidget(self.sensor_badge)
        header_layout.addWidget(self.stats_badge)
        header_layout.addWidget(self.mission_badge)
        header_layout.addWidget(self.save_map_btn)
        header_layout.addWidget(self.load_map_btn)
        header_layout.addWidget(self.fit_map_btn)
        header_layout.addWidget(self.reset_map_btn)
        header_layout.addWidget(self.back_btn)
        layout.addWidget(header)

        # Stacked Widget (Index 0: 2D Occupancy Grid, Index 1: 3D Point Cloud)
        self.stack = QStackedWidget()
        
        # 1. 2D Occupancy Grid Widget
        self.occupancy_widget = OccupancyGridWidget(self.map_manager)
        self.stack.addWidget(self.occupancy_widget)

        # 2. 3D OpenGL Viewport Widget
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

        self.drone_marker_3d = gl.GLScatterPlotItem(
            pos=np.empty((0, 3), dtype=np.float32),
            color=(0.95, 0.25, 0.25, 1.0),
            size=10.0,
            pxMode=True
        )
        self.gl_widget.addItem(self.drone_marker_3d)

        self.stack.addWidget(self.gl_widget)
        layout.addWidget(self.stack, 1)

        # Controlled Render Loop Timer (15 Hz)
        self.render_timer = QTimer(self)
        self.render_timer.setInterval(66) # 15 FPS
        self.render_timer.timeout.connect(self.render_tick)
        self.render_timer.start()

        self.last_3d_render_time = 0.0

        # Signals for non-blocking save/load
        self.save_status_signal.connect(self.handle_save_status)
        self.load_status_signal.connect(self.handle_load_status)

    def show_2d_mode(self):
        self.stack.setCurrentIndex(0)
        self.btn_2d_mode.setObjectName("btn_mode_active")
        self.btn_3d_mode.setObjectName("btn_mode_inactive")
        self.btn_2d_mode.style().unpolish(self.btn_2d_mode)
        self.btn_2d_mode.style().polish(self.btn_2d_mode)
        self.btn_3d_mode.style().unpolish(self.btn_3d_mode)
        self.btn_3d_mode.style().polish(self.btn_3d_mode)
        self.occupancy_widget.update()

    def show_3d_mode(self):
        self.stack.setCurrentIndex(1)
        self.btn_3d_mode.setObjectName("btn_mode_active")
        self.btn_2d_mode.setObjectName("btn_mode_inactive")
        self.btn_2d_mode.style().unpolish(self.btn_2d_mode)
        self.btn_2d_mode.style().polish(self.btn_2d_mode)
        self.btn_3d_mode.style().unpolish(self.btn_3d_mode)
        self.btn_3d_mode.style().polish(self.btn_3d_mode)
        self.update_3d_render()

    def render_tick(self):
        if not self.isVisible():
            return
        if self.stack.currentIndex() == 0:
            self.occupancy_widget.update()
        elif self.stack.currentIndex() == 1:
            now = time.time()
            if now - self.last_3d_render_time > 0.15: # At most 6.5 Hz for 3D OpenGL buffer uploads
                self.update_3d_render()
                self.last_3d_render_time = now

    @Slot(int, int, float, float, float, float)
    def handle_scan_processed(self, scan_pts, total_pts, dx, dy, dz, yaw_rad):
        # Update drone pose
        self.occupancy_widget.set_drone_pose(dx, dy, yaw_rad)

        if self.stack.currentIndex() == 1 and self.isVisible():
            drone_pos = np.array([[dx, dy, dz]], dtype=np.float32)
            self.drone_marker_3d.setData(pos=drone_pos)

        # Update stats badge
        explored_cells = len(self.map_manager.grid_cells)
        area_m2 = explored_cells * (self.map_manager.cell_size ** 2)
        self.stats_badge.setText(f"EXPLORED: {area_m2:.1f} m² | 3D PTS: {total_pts:,}")

    # Backward compatibility slot for tests
    def add_lidar_scan(self, pts_new, sensor_name, scan_pts, dx, dy, dz, yaw_rad=0.0):
        self.map_manager.add_lidar_scan(pts_new, dx, dy, dz)
        total_pts = len(self.map_manager.voxel_points)
        self.handle_scan_processed(scan_pts, total_pts, dx, dy, dz, yaw_rad)

    def add_point_cloud(self, pts_new, sensor_name, scan_pts, dx, dy, dz, yaw_rad=0.0):
        self.add_lidar_scan(pts_new, sensor_name, scan_pts, dx, dy, dz, yaw_rad)

    def update_3d_render(self):
        pts_3d = self.map_manager.get_3d_points()
        if len(pts_3d) > 0:
            colors = compute_point_colors(pts_3d)
            self.scatter_item.setData(pos=pts_3d, color=colors)

    @Slot(str, str)
    def update_mission_status(self, phase, msg):
        self.mission_badge.setText(f"MISSION: {phase}")
        self.statusBar().showMessage(f"[{phase}] {msg}") if self.statusBar() else None

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

    def fit_to_map(self):
        self.occupancy_widget.fit_to_map()

    def reset_view(self):
        self.occupancy_widget.reset_view()
        self.gl_widget.setCameraPosition(distance=35, elevation=25, azimuth=45)

    def save_map_action(self):
        self.save_map_btn.setEnabled(False)
        self.save_map_btn.setText("Saving...")
        def do_save():
            try:
                self.map_manager.save_map(MAP_SAVE_FILE)
                self.save_status_signal.emit(True, f"Map saved to:\n{MAP_SAVE_FILE}")
            except Exception as e:
                self.save_status_signal.emit(False, str(e))
        threading.Thread(target=do_save, daemon=True).start()

    @Slot(bool, str)
    def handle_save_status(self, success, msg):
        self.save_map_btn.setEnabled(True)
        self.save_map_btn.setText("Save Map")
        if success:
            QMessageBox.information(self, "Map Saved", msg)
        else:
            QMessageBox.critical(self, "Save Error", msg)

    def load_map_action(self):
        self.load_map_btn.setEnabled(False)
        self.load_map_btn.setText("Loading...")
        def do_load():
            try:
                ok = self.map_manager.load_map(MAP_SAVE_FILE)
                self.load_status_signal.emit(ok, "Loaded map successfully." if ok else "No saved map file found.")
            except Exception as e:
                self.load_status_signal.emit(False, str(e))
        threading.Thread(target=do_load, daemon=True).start()

    @Slot(bool, str)
    def handle_load_status(self, success, msg):
        self.load_map_btn.setEnabled(True)
        self.load_map_btn.setText("Load Map")
        if success:
            self.occupancy_widget.fit_to_map()
            self.update_3d_render()
            QMessageBox.information(self, "Map Loaded", msg)
        else:
            QMessageBox.warning(self, "Load Map", msg)

    def reset_map_action(self):
        reply = QMessageBox.question(
            self, "Reset Map",
            "Are you sure you want to clear the persistent map?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.map_manager.clear()
            self.scatter_item.setData(pos=np.empty((0, 3), dtype=np.float32))
            self.occupancy_widget.update()
            self.stats_badge.setText("EXPLORED: 0.0 m² | 3D PTS: 0")


# --- Final 3D Map Reconstruction Viewer Window ---

class FinalMapViewerWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.artifact = None
        self.setWindowTitle("LifeMAS — Final 3D Aerial Survey Reconstruction")
        self.resize(1280, 850)

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
        header_layout.setSpacing(10)

        title = QLabel("3D AERIAL SURVEY RECONSTRUCTION")
        title.setObjectName("map_title")

        self.survey_id_lbl = QLabel("SURVEY: NONE")
        self.survey_id_lbl.setObjectName("badge_neutral")

        self.pts_count_lbl = QLabel("SURFACE POINTS: 0")
        self.pts_count_lbl.setObjectName("badge_info")

        self.coverage_lbl = QLabel("COVERAGE: 0.0 m²")
        self.coverage_lbl.setObjectName("badge_info")

        self.status_badge = QLabel("● RECONSTRUCTION VERIFIED")
        self.status_badge.setObjectName("badge_active")

        # Action Buttons
        self.fit_map_btn = QPushButton("Fit to Map")
        self.fit_map_btn.setObjectName("btn_small")
        self.fit_map_btn.clicked.connect(self.fit_to_map)

        self.reset_view_btn = QPushButton("Reset View")
        self.reset_view_btn.setObjectName("btn_small")
        self.reset_view_btn.clicked.connect(self.reset_view)

        self.export_btn = QPushButton("Export Point Cloud...")
        self.export_btn.setObjectName("btn_small")
        self.export_btn.clicked.connect(self.export_map_action)

        self.load_survey_btn = QPushButton("Load Survey...")
        self.load_survey_btn.setObjectName("btn_small")
        self.load_survey_btn.clicked.connect(self.load_survey_action)

        self.close_btn = QPushButton("◀ Close / Return to Sim")
        self.close_btn.setObjectName("btn_primary_small")
        self.close_btn.clicked.connect(self.hide)

        header_layout.addWidget(title)
        header_layout.addWidget(self.survey_id_lbl)
        header_layout.addWidget(self.pts_count_lbl)
        header_layout.addWidget(self.coverage_lbl)
        header_layout.addWidget(self.status_badge)
        header_layout.addStretch()
        header_layout.addWidget(self.fit_map_btn)
        header_layout.addWidget(self.reset_view_btn)
        header_layout.addWidget(self.export_btn)
        header_layout.addWidget(self.load_survey_btn)
        header_layout.addWidget(self.close_btn)
        layout.addWidget(header)

        # 3D Viewport
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setBackgroundColor('#080a0e')
        self.gl_widget.setCameraPosition(distance=40, elevation=35, azimuth=45)

        self.grid_item = gl.GLGridItem()
        self.grid_item.setSize(x=100, y=100, z=0)
        self.grid_item.setSpacing(x=5, y=5, z=5)
        self.grid_item.setColor((0.15, 0.22, 0.3, 0.6))
        self.gl_widget.addItem(self.grid_item)

        self.axis_item = gl.GLAxisItem()
        self.axis_item.setSize(x=5, y=5, z=5)
        self.gl_widget.addItem(self.axis_item)

        self.scatter_item = gl.GLScatterPlotItem(
            pos=np.empty((0, 3), dtype=np.float32),
            color=np.empty((0, 4), dtype=np.float32),
            size=2.5,
            pxMode=True
        )
        self.gl_widget.addItem(self.scatter_item)

        layout.addWidget(self.gl_widget, 1)

    def load_artifact(self, artifact: SurveyArtifact):
        self.artifact = artifact
        if not artifact or not artifact.is_valid():
            return

        pts = artifact.points_3d
        cols = artifact.colors
        meta = artifact.metadata

        self.survey_id_lbl.setText(f"SURVEY: {meta.get('survey_id', 'Unknown')}")
        self.pts_count_lbl.setText(f"SURFACE POINTS: {len(pts):,}")
        area_m2 = meta.get('area_m2', 0.0)
        self.coverage_lbl.setText(f"COVERAGE: {area_m2:.1f} m²")
        self.setWindowTitle(f"LifeMAS — Final 3D Aerial Survey Reconstruction [{meta.get('survey_id', '')}]")

        self.scatter_item.setData(pos=pts, color=cols, size=2.5, pxMode=True)

        b_min = np.array(meta.get("bounds_min", [0, 0, 0]))
        b_max = np.array(meta.get("bounds_max", [0, 0, 0]))
        extent = b_max - b_min
        grid_sz = max(60, int(max(extent[0], extent[1]) * 1.5))
        self.grid_item.setSize(x=grid_sz, y=grid_sz, z=0)

        self.fit_to_map()

    def fit_to_map(self):
        if self.artifact and self.artifact.is_valid():
            b_min = np.array(self.artifact.metadata.get("bounds_min", [0, 0, 0]))
            b_max = np.array(self.artifact.metadata.get("bounds_max", [0, 0, 0]))
            center = (b_min + b_max) / 2.0
            extent = b_max - b_min
            max_span = max(float(extent[0]), float(extent[1]), float(extent[2]), 10.0)
            self.gl_widget.setCameraPosition(
                pos=QVector3D(float(center[0]), float(center[1]), float(center[2])),
                distance=max_span * 1.5,
                elevation=35,
                azimuth=45
            )

    def reset_view(self):
        self.gl_widget.setCameraPosition(distance=40, elevation=35, azimuth=45)

    def export_map_action(self):
        if not self.artifact or not self.artifact.is_valid():
            QMessageBox.warning(self, "Export", "No valid survey artifact loaded to export.")
            return

        filepath, selected_filter = QFileDialog.getSaveFileName(
            self, "Export Final 3D Point Cloud",
            f"survey_{self.artifact.metadata.get('survey_id', 'export')}.xyz",
            "XYZ Point Cloud (*.xyz);;PLY Format (*.ply);;NumPy Compressed (*.npz)"
        )
        if filepath:
            fmt = "xyz"
            if filepath.endswith(".ply"):
                fmt = "ply"
            elif filepath.endswith(".npz"):
                fmt = "npz"
            ok = export_survey_point_cloud(self.artifact, filepath, fmt)
            if ok:
                QMessageBox.information(self, "Export Successful", f"Survey point cloud exported to:\n{filepath}")
            else:
                QMessageBox.critical(self, "Export Failed", "Failed to write export file.")

    def load_survey_action(self):
        maps_dir = os.path.join(PROJECT, "data", "maps")
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Load Saved Final Survey Map", maps_dir, "Survey Artifacts (*.npz)"
        )
        if filepath:
            try:
                art = ReconstructionEngine.load_artifact(filepath)
                if art.is_valid():
                    self.load_artifact(art)
                    QMessageBox.information(self, "Survey Loaded", f"Loaded survey:\n{filepath}")
                else:
                    QMessageBox.warning(self, "Invalid File", "File does not contain a valid survey artifact.")
            except Exception as e:
                QMessageBox.critical(self, "Load Error", str(e))


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

        # Survey & Reconstruction State
        self.survey_worker = None
        self.reconstruction_worker = None
        self.survey_dataset = None
        self.survey_config = SurveyConfig()
        self.current_artifact = None

        if os.path.exists(FINAL_SURVEY_LATEST):
            try:
                art = ReconstructionEngine.load_artifact(FINAL_SURVEY_LATEST)
                if art.is_valid():
                    self.current_artifact = art
            except Exception:
                pass

        # Final 3D Map Viewer Window
        self.final_map_window = FinalMapViewerWindow(self)
        if self.current_artifact:
            self.final_map_window.load_artifact(self.current_artifact)

        # Persistent Map Manager (Preserved for live telemetry)
        self.map_manager = PersistentMapManager()
        
        # Load existing saved map if present
        if os.path.exists(MAP_SAVE_FILE):
            self.map_manager.load_map(MAP_SAVE_FILE)

        # Create 3D/2D Spatial Map Window (Hidden initially)
        self.map_window = SpatialMapWindow(self.map_manager, self)

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
        app_sub = QLabel("AERIAL LIDAR SURVEY & 3D RECONSTRUCTION ENGINE")
        app_sub.setObjectName("app_sub")
        title_box.addWidget(app_title)
        title_box.addWidget(app_sub)

        header.addLayout(title_box)
        header.addStretch()

        self.open_map_hdr_btn = QPushButton("Final 3D Map ↗")
        self.open_map_hdr_btn.setObjectName("btn_accent_hdr")
        self.open_map_hdr_btn.setEnabled(self.current_artifact is not None)
        self.open_map_hdr_btn.clicked.connect(self.open_final_map_window)
        self.hdr_final_map_btn = self.open_map_hdr_btn

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

        # MAIN WORKSPACE: Embedded AirSimNH Simulator Viewport
        self.main_sim_viewport = Viewport()
        self.main_sim_viewport.on_resize = self.resize_hosted_window
        vp_layout = QVBoxLayout(self.main_sim_viewport)
        vp_layout.setContentsMargins(0, 0, 0, 0)
        
        self.placeholder = QLabel(
            "LIVE AIRSIMNH SIMULATOR WORKSPACE\n\n"
            "1. Click 'Launch AirSimNH' in the sidebar\n"
            "2. Wait for window detection\n"
            "3. Click 'Embed Viewport' to host Unreal Engine view here\n"
            "4. Click 'Connect API' to begin telemetry & persistent mapping"
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

        # Section 2: Aerial Survey Mission Workflow
        sidebar_layout.addWidget(self.section_header("2. AERIAL SURVEY MISSION"))

        self.workflow_badge = QLabel("● WORKFLOW: READY")
        self.workflow_badge.setObjectName("badge_status_neutral")
        self.workflow_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.survey_progress_bar = QProgressBar()
        self.survey_progress_bar.setRange(0, 100)
        self.survey_progress_bar.setValue(0)
        self.survey_progress_bar.setFixedHeight(12)
        self.survey_progress_bar.setTextVisible(True)
        self.survey_progress_bar.setStyleSheet("""
            QProgressBar {
                background-color: #181d27; border: 1px solid #1e2430; border-radius: 3px;
                color: #e2e8f0; font-size: 9px; text-align: center; font-weight: bold;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #2563eb, stop:1 #06b6d4);
                border-radius: 2px;
            }
        """)

        self.start_survey_btn = QPushButton("▶ Start Aerial Survey")
        self.start_survey_btn.setObjectName("btn_mission_start")
        self.start_survey_btn.clicked.connect(self.start_aerial_survey)

        survey_ctrl_row = QHBoxLayout()
        survey_ctrl_row.setSpacing(6)
        self.pause_survey_btn = QPushButton("Pause / Hover")
        self.pause_survey_btn.setObjectName("btn_secondary")
        self.pause_survey_btn.setEnabled(False)
        self.pause_survey_btn.clicked.connect(self.pause_aerial_survey)

        self.abort_survey_btn = QPushButton("Land / Abort")
        self.abort_survey_btn.setObjectName("btn_secondary")
        self.abort_survey_btn.setEnabled(False)
        self.abort_survey_btn.clicked.connect(self.abort_aerial_survey)

        survey_ctrl_row.addWidget(self.pause_survey_btn)
        survey_ctrl_row.addWidget(self.abort_survey_btn)

        self.mission_status_lbl = QLabel("READY: Drone idle on ground. Click Start Survey.")
        self.mission_status_lbl.setObjectName("telem_name")
        self.mission_status_lbl.setWordWrap(True)

        # Backward compatibility aliases for existing tests
        self.start_mission_btn = self.start_survey_btn
        self.pause_mission_btn = self.pause_survey_btn
        self.land_btn = self.abort_survey_btn

        sidebar_layout.addWidget(self.workflow_badge)
        sidebar_layout.addWidget(self.survey_progress_bar)
        sidebar_layout.addWidget(self.start_survey_btn)
        sidebar_layout.addLayout(survey_ctrl_row)
        sidebar_layout.addWidget(self.mission_status_lbl)

        # Section 3: Final 3D Reconstruction
        sidebar_layout.addWidget(self.section_header("3. FINAL 3D RECONSTRUCTION"))

        self.final_map_btn = QPushButton("▶ OPEN FINAL 3D MAP ↗")
        self.final_map_btn.setObjectName("btn_final_map")
        self.final_map_btn.setEnabled(False) # MUST START DISABLED!
        self.final_map_btn.clicked.connect(self.open_final_map_window)
        self.open_map_card_btn = self.final_map_btn # Alias for compatibility

        reconstruction_row = QHBoxLayout()
        self.load_prev_survey_btn = QPushButton("Load Saved Survey...")
        self.load_prev_survey_btn.setObjectName("btn_small")
        self.load_prev_survey_btn.clicked.connect(self.load_previous_survey)
        reconstruction_row.addWidget(self.load_prev_survey_btn)

        self.sidebar_sensor_lbl = QLabel("Mode: 3D Survey")
        self.sidebar_sensor_lbl.setObjectName("telem_val")
        reconstruction_row.addWidget(self.sidebar_sensor_lbl)

        self.sidebar_pts_lbl = QLabel("Explored: 0.0 m²")
        self.sidebar_pts_lbl.setObjectName("telem_val")

        self.final_stats_lbl = QLabel("No completed survey.")
        self.final_stats_lbl.setObjectName("telem_name")
        self.final_stats_lbl.setWordWrap(True)

        if self.current_artifact:
            pts_cnt = len(self.current_artifact.points_3d)
            area = self.current_artifact.metadata.get("area_m2", 0.0)
            self.final_stats_lbl.setText(f"Previous: {self.current_artifact.metadata.get('survey_id', '')} ({pts_cnt:,} pts · {area:.1f} m²)")
            self.load_prev_survey_btn.setText("View Previous Survey ↗")

        sidebar_layout.addWidget(self.final_map_btn)
        sidebar_layout.addLayout(reconstruction_row)
        sidebar_layout.addWidget(self.final_stats_lbl)

        # Section 4: Telemetry
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

        # Map Periodic Auto-Save Timer (Every 15 Seconds, Asynchronous)
        self.autosave_timer = QTimer(self)
        self.autosave_timer.setInterval(15000)
        self.autosave_timer.timeout.connect(self.autosave_map)
        self.autosave_timer.start()

    @staticmethod
    def section_header(text):
        lbl = QLabel(text)
        lbl.setObjectName("section_hdr")
        return lbl

    def autosave_map(self):
        if self.map_manager.has_bounds or len(self.map_manager.grid_cells) > 0:
            def do_save():
                try:
                    self.map_manager.save_map(MAP_SAVE_FILE)
                except Exception:
                    pass
            threading.Thread(target=do_save, daemon=True).start()

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
            QLabel#badge_status_info {
                background: #1e3a8a; color: #93c5fd;
                padding: 5px 10px; border-radius: 4px; font-weight: 600; font-size: 11px;
            }
            QLabel#badge_status_warning {
                background: #78350f; color: #fbbf24;
                padding: 5px 10px; border-radius: 4px; font-weight: 600; font-size: 11px;
            }
            QLabel#badge_status_purple {
                background: #4c1d95; color: #c4b5fd;
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
            QPushButton#btn_final_map {
                background: #059669; color: #ffffff; border: none;
                font-weight: 700; font-size: 12px; padding: 10px; border-radius: 4px;
            }
            QPushButton#btn_final_map:hover { background: #10b981; }
            QPushButton#btn_final_map:disabled {
                background: #141a24; color: #475569; border: 1px solid #1e2430;
            }

            QPushButton#btn_mode_active {
                background: #2563eb; color: #ffffff; border: none; font-weight: 700; padding: 4px 10px;
            }
            QPushButton#btn_mode_inactive {
                background: #1e293b; color: #94a3b8; border: 1px solid #334155; padding: 4px 10px;
            }
            QPushButton#btn_mode_inactive:hover { background: #334155; color: #ffffff; }

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

        self.worker = LidarTelemetryWorker(self.map_manager)
        self.worker.set_vehicle(self.selected_vehicle)
        self.worker.scan_processed_signal.connect(self.map_window.handle_scan_processed)
        self.worker.scan_processed_signal.connect(self.update_sidebar_map_stats)
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

    def update_sidebar_map_stats(self, *args, **kwargs):
        explored_cells = len(self.map_manager.grid_cells)
        area_m2 = explored_cells * (self.map_manager.cell_size ** 2)
        self.sidebar_pts_lbl.setText(f"Explored: {area_m2:.1f} m²")

    @Slot(dict)
    def update_telemetry_ui(self, t):
        vehicles_str = ", ".join(t["active_vehicles"]) if t["active_vehicles"] else "None"
        self.telemetry["Active Vehicle"].setText(vehicles_str)
        self.telemetry["Position X/Y"].setText(f"{t['pos_x']:.1f}, {t['pos_y']:.1f}")
        self.telemetry["Altitude"].setText(f"{t['altitude']:.2f} m")
        self.telemetry["Velocity"].setText(f"{t['vel_x']:.1f}, {t['vel_y']:.1f}, {t['vel_z']:.1f}")
        self.telemetry["Ready / Arm"].setText(f"R:{t['ready']} | Arm:{t['can_arm']}")
        self.telemetry["Landed State"].setText(t["landed_state"].replace("LandedState.", ""))

    # --- Mission Control Methods ---

    def set_workflow_state(self, state: str, message: str, progress: int = 0):
        self.workflow_badge.setText(f"● {state.upper()}")
        badge_name = "badge_status_neutral"
        st_upper = state.upper()
        if "READY" in st_upper:
            badge_name = "badge_status_neutral"
        elif "SURVEY" in st_upper or "TAKEOFF" in st_upper:
            badge_name = "badge_status_info"
        elif "RETURN" in st_upper or "LAND" in st_upper or "PAUSE" in st_upper:
            badge_name = "badge_status_warning"
        elif "PROCESS" in st_upper:
            badge_name = "badge_status_purple"
        elif "FINAL MAP" in st_upper:
            badge_name = "badge_status_active"
        elif "FAIL" in st_upper or "ERROR" in st_upper or "STOP" in st_upper:
            badge_name = "badge_status_warning"

        self.workflow_badge.setObjectName(badge_name)
        self.workflow_badge.style().unpolish(self.workflow_badge)
        self.workflow_badge.style().polish(self.workflow_badge)

        self.mission_status_lbl.setText(f"{state}: {message}")
        if progress >= 0:
            self.survey_progress_bar.setValue(min(100, progress))
        self.status.showMessage(f"[{state}] {message}")

    def start_aerial_survey(self):
        # Auto-connect API if not already connected
        if not self.api_connected or self.worker is None or not self.worker.isRunning():
            self.connect_api()

        # Disable Final Map button for new survey run
        self.final_map_btn.setEnabled(False)
        self.hdr_final_map_btn.setEnabled(False)

        # Initialize survey dataset
        self.survey_config = SurveyConfig(
            survey_id=time.strftime("survey_%Y%m%d_%H%M%S"),
            length_x=14.0,
            width_y=8.0,
            altitude=3.8,
            lane_spacing=2.5,
            flight_speed=1.2,
            turn_speed=0.8,
            min_scans_required=15,
            voxel_size=0.08,
            min_range=0.40,
            max_range=45.0
        )
        self.survey_dataset = SurveyDataset(self.survey_config.survey_id, self.survey_config)
        if self.worker:
            self.worker.set_survey_dataset(self.survey_dataset)

        self.start_survey_btn.setEnabled(False)
        self.pause_survey_btn.setEnabled(True)
        self.pause_survey_btn.setText("Pause / Hover")
        self.abort_survey_btn.setEnabled(True)
        self.set_workflow_state("SURVEYING", "Initiating aerial survey mission...", 5)

        self.survey_worker = SurveyMissionWorker(
            vehicle_name=self.selected_vehicle,
            survey_config=self.survey_config
        )
        self.survey_worker.status_signal.connect(self.handle_survey_flight_status)
        self.survey_worker.completed_signal.connect(self.handle_survey_flight_completed)
        self.survey_worker.start()

    def pause_aerial_survey(self):
        if self.survey_worker and self.survey_worker.isRunning():
            if not self.survey_worker.paused:
                self.survey_worker.pause()
                self.pause_survey_btn.setText("Resume Survey")
                self.set_workflow_state("PAUSED", "Survey paused. Hovering in place.", -1)
            else:
                self.survey_worker.resume()
                self.pause_survey_btn.setText("Pause / Hover")
                self.set_workflow_state("SURVEYING", "Resuming survey passes...", -1)

    def abort_aerial_survey(self):
        self.set_workflow_state("RETURNING / LANDING", "Survey abort requested. Descending safely...", 90)
        self.pause_survey_btn.setEnabled(False)
        self.abort_survey_btn.setEnabled(False)
        if self.survey_worker and self.survey_worker.isRunning():
            self.survey_worker.cancel()

    @Slot(str, str, int)
    def handle_survey_flight_status(self, phase, msg, pct):
        self.set_workflow_state(phase, msg, pct)
        if hasattr(self, "map_window") and self.map_window:
            self.map_window.update_mission_status(phase, msg)

    @Slot(bool, str)
    def handle_survey_flight_completed(self, success, msg):
        self.pause_survey_btn.setEnabled(False)
        self.pause_survey_btn.setText("Pause / Hover")
        self.abort_survey_btn.setEnabled(False)
        self.start_survey_btn.setEnabled(True)

        if not success:
            self.set_workflow_state("FAILED", f"Flight ended: {msg}", 0)
            self.final_stats_lbl.setText(f"Flight aborted / error: {msg}")
            return

        scan_count = self.survey_dataset.get_scan_count() if self.survey_dataset else 0
        if scan_count < self.survey_config.min_scans_required:
            self.set_workflow_state(
                "FAILED",
                f"Insufficient LiDAR data ({scan_count} scans < {self.survey_config.min_scans_required}). Final map unavailable.",
                0
            )
            self.final_stats_lbl.setText(f"Insufficient scans ({scan_count} < {self.survey_config.min_scans_required}).")
            return

        # Commence Post-Flight 3D Reconstruction
        self.start_survey_btn.setEnabled(False)
        self.set_workflow_state("PROCESSING", f"Processing {scan_count} recorded LiDAR scans...", 88)

        maps_dir = os.path.join(PROJECT, "data", "maps")

        # Safely preserve raw survey dataset to disk
        try:
            raw_filename = f"raw_survey_{self.survey_dataset.survey_id}.npz"
            raw_filepath = os.path.join(maps_dir, raw_filename)
            self.survey_dataset.save_raw_dataset(raw_filepath)
        except Exception as e:
            print(f"[SURVEY] Note: Could not save raw dataset: {e}")

        self.reconstruction_worker = MapReconstructionWorker(self.survey_dataset, maps_dir)
        self.reconstruction_worker.progress_signal.connect(self.handle_reconstruction_progress)
        self.reconstruction_worker.finished_signal.connect(self.handle_reconstruction_finished)
        self.reconstruction_worker.start()

    @Slot(int, str)
    def handle_reconstruction_progress(self, pct, msg):
        self.set_workflow_state("PROCESSING", msg, pct)

    @Slot(bool, object, str)
    def handle_reconstruction_finished(self, success, artifact, error_msg):
        self.start_survey_btn.setEnabled(True)

        if not success or artifact is None or not artifact.is_valid():
            self.set_workflow_state("FAILED", f"Reconstruction failed: {error_msg}", 0)
            self.final_map_btn.setEnabled(False)
            self.hdr_final_map_btn.setEnabled(False)
            self.final_stats_lbl.setText(f"Reconstruction error: {error_msg}")
            QMessageBox.critical(self, "Reconstruction Error", f"Processing failed:\n{error_msg}")
            return

        # Verification Succeeded!
        self.current_artifact = artifact
        self.set_workflow_state("FINAL MAP READY", "Final 3D reconstruction verified & saved! Click to view.", 100)
        
        # Enable Final Map button!
        self.final_map_btn.setEnabled(True)
        self.hdr_final_map_btn.setEnabled(True)

        pts_count = len(artifact.points_3d)
        area_m2 = artifact.metadata.get("area_m2", 0.0)
        self.final_stats_lbl.setText(f"Latest: {artifact.metadata.get('survey_id', '')}\n{pts_count:,} pts · {area_m2:.1f} m²")

        self.final_map_window.load_artifact(artifact)
        self.status.showMessage("3D Aerial survey reconstruction complete and verified!")

    def open_final_map_window(self):
        if self.current_artifact and self.current_artifact.is_valid():
            self.final_map_window.load_artifact(self.current_artifact)
            self.final_map_window.showMaximized()
            self.final_map_window.raise_()
            self.final_map_window.activateWindow()
        else:
            QMessageBox.warning(
                self, "Final Map Unavailable",
                "No verified 3D survey reconstruction is available yet.\n"
                "Please run an aerial survey or load a saved survey file."
            )

    def load_previous_survey(self):
        maps_dir = os.path.join(PROJECT, "data", "maps")
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Load Saved Aerial Survey",
            maps_dir, "Survey Artifacts (*.npz)"
        )
        if filepath:
            try:
                art = ReconstructionEngine.load_artifact(filepath)
                if art.is_valid():
                    self.current_artifact = art
                    self.final_map_window.load_artifact(art)
                    self.final_map_btn.setEnabled(True)
                    self.hdr_final_map_btn.setEnabled(True)
                    pts_count = len(art.points_3d)
                    area_m2 = art.metadata.get("area_m2", 0.0)
                    self.final_stats_lbl.setText(f"Loaded: {art.metadata.get('survey_id', '')}\n{pts_count:,} pts · {area_m2:.1f} m²")
                    self.set_workflow_state("FINAL MAP READY", "Loaded verified 3D survey artifact from disk.", 100)
                    self.open_final_map_window()
                else:
                    QMessageBox.warning(self, "Invalid File", "Selected file is not a valid survey artifact.")
            except Exception as e:
                QMessageBox.critical(self, "Load Error", f"Failed to load survey: {e}")

    # Aliases for backward compatibility with previous test suites
    def start_mapping_mission(self):
        self.start_aerial_survey()

    def pause_mapping_mission(self):
        self.pause_aerial_survey()

    def land_drone(self):
        self.abort_aerial_survey()

    @Slot(str, str)
    def handle_mission_status(self, phase, msg):
        self.set_workflow_state(phase, msg, -1)

    @Slot(bool, str)
    def finish_mapping_mission(self, success, msg):
        self.handle_survey_flight_completed(success, msg)

    @Slot(bool, str)
    def finish_landing(self, success, msg):
        self.handle_survey_flight_completed(success, msg)

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
        self.autosave_map()
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
