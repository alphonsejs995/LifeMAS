import ctypes
import os
import subprocess
import sys
import airsim
from ctypes import wintypes

from PySide6.QtCore import Qt, QTimer, QThread, Signal
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QStatusBar, QMessageBox, QComboBox
)

PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIMULATOR = os.path.join(
    PROJECT, "AirSimNH", "AirSimNH",
    "WindowsNoEditor", "AirSimNH.exe"
)

user32 = ctypes.WinDLL("user32", use_last_error=True)
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

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
        user32.GetWindowThreadProcessId(
            hwnd, ctypes.byref(window_pid)
        )

        candidates.append((int(hwnd), window_pid.value, title))
        return True

    user32.EnumWindows(callback, 0)

    # Prefer the window owned by the launched process.
    for hwnd, window_pid, title in candidates:
        if window_pid == pid:
            return hwnd

    # Unreal may create the visible window in another process.
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



class FlightManeuverWorker(QThread):
    status = Signal(str)
    completed = Signal(bool, str)

    def __init__(self, drone):
        super().__init__()
        self.drone = drone

    def run(self):
        client = None
        try:
            client = airsim.MultirotorClient(
                ip="127.0.0.1", port=41451, timeout_value=10
            )
            client.confirmConnection()
            client.enableApiControl(True, vehicle_name=self.drone)

            self.status.emit("Arming drone...")
            if not client.armDisarm(True, vehicle_name=self.drone):
                raise RuntimeError("AirSim refused to arm the drone.")

            self.status.emit("Taking off...")
            client.takeoffAsync(
                timeout_sec=15, vehicle_name=self.drone
            ).join()

            self.status.emit("Moving forward briefly...")
            client.moveByVelocityBodyFrameAsync(
                1.5, 0, 0, 2.0,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(False, 0),
                vehicle_name=self.drone,
            ).join()

            self.status.emit("Hovering briefly...")
            client.hoverAsync(vehicle_name=self.drone).join()
            import time
            time.sleep(1.0)

            self.status.emit("Landing...")
            client.landAsync(
                timeout_sec=30, vehicle_name=self.drone
            ).join()

            state = client.getMultirotorState(
                vehicle_name=self.drone
            )
            if state.landed_state != airsim.LandedState.Landed:
                self.status.emit("Still airborne; retrying landing...")
                client.landAsync(
                    timeout_sec=30, vehicle_name=self.drone
                ).join()
                state = client.getMultirotorState(
                    vehicle_name=self.drone
                )

            if state.landed_state == airsim.LandedState.Landed:
                client.armDisarm(False, vehicle_name=self.drone)
                self.completed.emit(
                    True, "Maneuver completed: takeoff, forward flight, landing."
                )
            else:
                self.completed.emit(
                    False, "Landing was not confirmed. Check the simulator."
                )

        except Exception as exc:
            self.completed.emit(False, f"Flight maneuver failed: {exc}")


class LifeMAS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LifeMAS | Drone Operations Center")
        self.resize(1440, 860)
        self.setMinimumSize(1000, 650)

        self.process = None
        self.api_client = None
        self.held_keys = set()
        self.flight_speed = 3.0
        self.yaw_speed = 45.0
        self.selected_vehicle = "Drone1"
        self.vehicle_names = ["Drone1", "Drone2", "Drone3"]
        self.sim_hwnd = None
        self.original_style = None
        self.original_parent = None
        self.wait_count = 0

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(22, 18, 22, 14)
        outer.setSpacing(16)

        header = QHBoxLayout()
        brand = QVBoxLayout()
        title = QLabel("LIFEMAS")
        title.setObjectName("brand")
        subtitle = QLabel("MULTI-AGENT DRONE OPERATIONS")
        subtitle.setObjectName("muted")
        brand.addWidget(title)
        brand.addWidget(subtitle)
        header.addLayout(brand)
        header.addStretch()

        self.connection = QLabel("●  SIMULATOR NOT CONNECTED")
        self.connection.setObjectName("status")
        header.addWidget(self.connection)
        outer.addLayout(header)

        body = QHBoxLayout()
        body.setSpacing(16)
        outer.addLayout(body, 1)

        left = QVBoxLayout()
        left.setSpacing(10)
        viewport_heading = QHBoxLayout()
        viewport_heading.addWidget(QLabel("SIMULATOR VIEWPORT"))
        viewport_heading.addStretch()
        viewport_heading.addWidget(QLabel("AIRSIMNH"))
        left.addLayout(viewport_heading)

        self.viewport = Viewport()
        self.viewport.on_resize = self.resize_hosted_window
        vp_layout = QVBoxLayout(self.viewport)
        self.placeholder = QLabel(
            "SIMULATOR VIEWPORT\n\n"
            "1. Launch AirSimNH below\n"
            "2. Wait for the world to load\n"
            "3. Embed the live window here\n\n"
            "Native Unreal window hosting is experimental."
        )
        self.placeholder.setObjectName("placeholder")
        self.placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.placeholder.setWordWrap(True)
        vp_layout.addWidget(self.placeholder)
        left.addWidget(self.viewport, 1)

        controls = QHBoxLayout()
        self.launch_btn = QPushButton("1   Launch AirSimNH")
        self.launch_btn.setObjectName("primary")
        self.launch_btn.clicked.connect(self.launch_simulator)
        self.embed_btn = QPushButton("2   Embed live window")
        self.embed_btn.clicked.connect(self.embed_window)
        self.embed_btn.setEnabled(False)
        self.restore_btn = QPushButton("Restore window")
        self.restore_btn.clicked.connect(self.restore_window)
        self.restore_btn.setEnabled(False)
        self.connect_api_btn = QPushButton("3   Connect API")
        self.connect_api_btn.clicked.connect(self.connect_api)
        controls.addWidget(self.launch_btn)
        controls.addWidget(self.embed_btn)
        controls.addWidget(self.restore_btn)
        controls.addWidget(self.connect_api_btn)
        left.addLayout(controls)
        body.addLayout(left, 7)

        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setMinimumWidth(255)
        sidebar.setMaximumWidth(320)
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(18, 18, 18, 18)
        side.setSpacing(13)

        side.addWidget(QLabel("SYSTEM OVERVIEW"))
        self.sim_state = QLabel("●  Not launched")
        side.addWidget(self.sim_state)

        side.addWidget(self.section_label("SELECTED DRONE"))
        self.vehicle_selector = QComboBox()
        self.vehicle_selector.addItems(self.vehicle_names)
        self.vehicle_selector.currentTextChanged.connect(self.select_vehicle)
        side.addWidget(self.vehicle_selector)

        self.maneuver_btn = QPushButton("Run Basic Flight Maneuver")
        self.maneuver_btn.setObjectName("primary")
        self.maneuver_btn.clicked.connect(self.start_flight_maneuver)
        side.addWidget(self.maneuver_btn)

        side.addWidget(self.section_label("DRONE TELEMETRY"))
        grid = QGridLayout()
        grid.setVerticalSpacing(12)
        self.telemetry = {}
        for row, (name, value) in enumerate([
            ("Active drones", "0"),
            ("Altitude", "—"),
            ("Position", "—"),
            ("Flight mode", "Unknown"),
            ("Ready", "Not checked"),
            ("Can arm", "Not checked"),
        ]):
            grid.addWidget(QLabel(name), row, 0)
            val = QLabel(value)
            val.setObjectName("value")
            grid.addWidget(val, row, 1)
            self.telemetry[name] = val
        side.addLayout(grid)

        side.addWidget(self.section_label("KEYBOARD CONTROLS"))
        side.addWidget(QLabel("W / S     Forward / backward"))
        side.addWidget(QLabel("A / D     Left / right"))
        side.addWidget(QLabel("Q / E     Down / up"))
        side.addWidget(QLabel("Hold WASD to fly ? R/F altitude ? Q/E yaw"))
        side.addWidget(self.section_label("PROJECT MODULES"))
        for name in [
            "01   Simulator integration",
            "02   Manual flight controls",
            "03   Environment mapping",
            "04   Multi-agent coordination",
            "05   MARL training",
        ]:
            item = QLabel(name)
            item.setObjectName("module")
            side.addWidget(item)

        side.addStretch()
        note = QLabel(
            "FLIGHT CONTROLS\n"
            "Click the dashboard, then use WASD.\n"
            "Release keys to stop movement."
        )
        note.setObjectName("muted")
        note.setWordWrap(True)
        side.addWidget(note)
        body.addWidget(sidebar, 2)

        status = QStatusBar()
        status.showMessage("Ready · UI prototype · Flight controls disabled")
        self.setStatusBar(status)
        self.status = status

        self.setStyleSheet("""
            QWidget#root {
                background: #0b1020; color: #e5eaf5;
                font-family: Segoe UI; font-size: 13px;
            }
            QLabel#brand {
                font-size: 27px; font-weight: 800;
                letter-spacing: 3px; color: #72e0c0;
            }
            QLabel#muted { color: #8793aa; }
            QLabel#status {
                color: #f0bd72; background: #242034;
                padding: 10px 14px; border-radius: 8px;
            }
            QFrame#viewport {
                background: #050811; border: 1px solid #303b51;
                border-radius: 10px;
            }
            QLabel#placeholder {
                color: #8493ad; font-size: 15px;
                background: transparent; line-height: 1.5;
            }
            QFrame#sidebar {
                background: #131b2d; border: 1px solid #26344a;
                border-radius: 10px;
            }
            QLabel#value {
                color: #72e0c0; font-weight: 600;
            }
            QLabel#module {
                padding: 9px; background: #1b263a;
                border-radius: 6px; color: #cbd5e6;
            }
            QPushButton {
                color: #dce5f5; background: #202c42;
                border: 1px solid #36465f; padding: 12px;
                border-radius: 7px; font-weight: 600;
            }
            QPushButton:hover { background: #2b3a54; }
            QPushButton:disabled { color: #67748a; }
            QPushButton#primary {
                background: #147d6c; border: none; color: white;
            }
            QPushButton#primary:hover { background: #199780; }
            QStatusBar {
                background: #10182a; color: #94a3b8;
            }
        """)

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(1000)
        self.poll_timer.timeout.connect(self.check_simulator_window)

        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.setInterval(500)
        self.telemetry_timer.timeout.connect(self.update_telemetry)

        self.flight_timer = QTimer(self)
        self.flight_timer.setInterval(50)
        self.flight_timer.timeout.connect(self.send_flight_command)
        self.flight_timer.start()

    @staticmethod
    def section_label(text):
        label = QLabel(text)
        label.setStyleSheet(
            "color: #72e0c0; font-weight: 700; margin-top: 12px;"
        )
        return label

    def connect_api(self):
        try:
            client = airsim.MultirotorClient(
                ip="127.0.0.1", port=41451, timeout_value=5
            )
            client.confirmConnection()
            vehicles = client.listVehicles()
            if vehicles:
                self.vehicle_names = vehicles
                self.vehicle_selector.blockSignals(True)
                self.vehicle_selector.clear()
                self.vehicle_selector.addItems(vehicles)
                if self.selected_vehicle in vehicles:
                    self.vehicle_selector.setCurrentText(self.selected_vehicle)
                else:
                    self.selected_vehicle = vehicles[0]
                    self.vehicle_selector.setCurrentIndex(0)
                self.vehicle_selector.blockSignals(False)
            client.getMultirotorState(vehicle_name=self.selected_vehicle)

            self.api_client = client
            client.enableApiControl(True, vehicle_name=self.selected_vehicle)
            self.connect_api_btn.setEnabled(False)
            self.telemetry_timer.start()
            self.connection.setText("●  AIRSIM API CONNECTED")
            self.connection.setStyleSheet("color: #72e0c0;")
            self.status.showMessage("Connected. Read-only telemetry active.")
            self.update_telemetry()
        except Exception as exc:
            QMessageBox.warning(self, "API connection failed", str(exc))

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in (
            Qt.Key.Key_W, Qt.Key.Key_A, Qt.Key.Key_S, Qt.Key.Key_D,
            Qt.Key.Key_R, Qt.Key.Key_F, Qt.Key.Key_Q, Qt.Key.Key_E,
        ):
            self.held_keys.add(key)
            event.accept()
            return
        if key == Qt.Key.Key_Space:
            self.held_keys.clear()
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        self.held_keys.discard(event.key())
        super().keyReleaseEvent(event)

    def send_flight_command(self):
        if self.api_client is None:
            return

        keys = self.held_keys
        vx = self.flight_speed * (
            int(Qt.Key.Key_W in keys) - int(Qt.Key.Key_S in keys)
        )
        vy = self.flight_speed * (
            int(Qt.Key.Key_D in keys) - int(Qt.Key.Key_A in keys)
        )
        vz = self.flight_speed * (
            int(Qt.Key.Key_F in keys) - int(Qt.Key.Key_R in keys)
        )
        yaw = self.yaw_speed * (
            int(Qt.Key.Key_E in keys) - int(Qt.Key.Key_Q in keys)
        )

        if not keys:
            return

        try:
            self.api_client.moveByVelocityAsync(
                vx, vy, vz, 0.15,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(True, yaw),
                vehicle_name=self.selected_vehicle,
            )
        except Exception as exc:
            self.status.showMessage(f"Flight command failed: {exc}")

    def start_flight_maneuver(self):
        if self.api_client is None:
            QMessageBox.information(
                self, "Connect API first",
                "Click Connect API before starting the maneuver."
            )
            return

        if hasattr(self, "maneuver_worker") and self.maneuver_worker.isRunning():
            return

        self.maneuver_btn.setEnabled(False)
        self.status.showMessage("Starting basic flight maneuver...")
        self.maneuver_worker = FlightManeuverWorker(self.selected_vehicle)
        self.maneuver_worker.status.connect(self.status.showMessage)
        self.maneuver_worker.completed.connect(self.finish_flight_maneuver)
        self.maneuver_worker.start()

    def finish_flight_maneuver(self, success, message):
        self.maneuver_btn.setEnabled(True)
        self.status.showMessage(message)
        if success:
            QMessageBox.information(self, "Flight maneuver", message)
        else:
            QMessageBox.warning(self, "Flight maneuver", message)

    def select_vehicle(self, vehicle_name):
        if vehicle_name:
            self.selected_vehicle = vehicle_name
            self.status.showMessage(f"Selected vehicle: {vehicle_name}")
            self.update_telemetry()

    def update_telemetry(self):
        if self.api_client is None:
            return
        try:
            state = self.api_client.getMultirotorState(
                vehicle_name=self.selected_vehicle
            )
            pos = state.kinematics_estimated.position
            vel = state.kinematics_estimated.linear_velocity

            self.telemetry["Active drones"].setText(
                str(len(self.api_client.listVehicles()))
            )
            self.telemetry["Altitude"].setText(f"{-pos.z_val:.2f} m*")
            self.telemetry["Position"].setText(
                f"X {pos.x_val:.1f}  Y {pos.y_val:.1f}"
            )
            self.telemetry["Flight mode"].setText(
                f"V {vel.x_val:.1f}, {vel.y_val:.1f}, {vel.z_val:.1f}"
            )
            self.telemetry["Ready"].setText(str(state.ready))
            self.telemetry["Can arm"].setText(str(state.can_arm))
        except Exception as exc:
            self.telemetry_timer.stop()
            self.status.showMessage(f"Telemetry error: {exc}")

    def launch_simulator(self):
        if not os.path.isfile(SIMULATOR):
            QMessageBox.critical(
                self, "Simulator not found",
                "Could not find AirSimNH.exe at:\n\n" + SIMULATOR
            )
            return

        if self.process and self.process.poll() is None:
            self.status.showMessage(
                "AirSimNH was already launched from this dashboard."
            )
            return

        try:
            self.process = subprocess.Popen(
                [SIMULATOR],
                cwd=os.path.dirname(SIMULATOR)
            )
        except Exception as exc:
            QMessageBox.critical(self, "Launch failed", str(exc))
            return

        self.wait_count = 0
        self.launch_btn.setEnabled(False)
        self.sim_state.setText("●  Starting simulator...")
        self.status.showMessage(
            "Waiting for the AirSimNH window. Initial loading may take a while."
        )
        self.poll_timer.start()

    def check_simulator_window(self):
        self.wait_count += 1
        if self.process and self.process.poll() is not None:
            self.poll_timer.stop()
            self.launch_btn.setEnabled(True)
            self.sim_state.setText("●  Simulator process exited")
            self.status.showMessage(
                "AirSimNH exited. Check the simulator window for an error."
            )
            return

        if self.process:
            hwnd = find_window_for_pid(self.process.pid)
            if hwnd:
                self.sim_hwnd = hwnd
                self.poll_timer.stop()
                self.embed_btn.setEnabled(True)
                self.sim_state.setText("●  Window detected")
                self.connection.setText("●  SIMULATOR WINDOW FOUND")
                self.connection.setStyleSheet("color: #72e0c0;")
                self.status.showMessage(
                    "Window detected. Click Embed live window to test hosting."
                )
                return

        if self.wait_count >= 120:
            self.poll_timer.stop()
            self.launch_btn.setEnabled(True)
            self.status.showMessage(
                "No main window detected yet. Check whether AirSimNH finished loading."
            )

    def embed_window(self):
        if not self.sim_hwnd or not user32.IsWindow(self.sim_hwnd):
            QMessageBox.warning(
                self, "Window unavailable",
                "The simulator window was not found. Try launching it again."
            )
            return

        hwnd = self.sim_hwnd
        parent = int(self.viewport.winId())
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

        self.viewport.hosted_hwnd = hwnd
        self.placeholder.hide()
        self.restore_btn.setEnabled(True)
        self.embed_btn.setEnabled(False)
        self.connection.setText("●  EMBED TEST ACTIVE")
        self.status.showMessage(
            "Window reparented. Check rendering and mouse/keyboard behaviour."
        )

    def resize_hosted_window(self):
        if self.sim_hwnd and user32.IsWindow(self.sim_hwnd):
            user32.SetWindowPos(
                self.sim_hwnd, 0, 0, 0,
                self.viewport.width(), self.viewport.height(),
                SWP_NOZORDER | SWP_FRAMECHANGED | SWP_SHOWWINDOW
            )

    def restore_window(self):
        if self.sim_hwnd and user32.IsWindow(self.sim_hwnd):
            user32.SetParent(self.sim_hwnd, None)
            if self.original_style is not None:
                set_style(
                    self.sim_hwnd, GWL_STYLE, self.original_style
                )
            user32.SetWindowPos(
                self.sim_hwnd, 0, 100, 100, 1280, 720,
                SWP_NOZORDER | SWP_FRAMECHANGED | SWP_SHOWWINDOW
            )
            user32.ShowWindow(self.sim_hwnd, SW_RESTORE)

        self.viewport.hosted_hwnd = None
        self.placeholder.show()
        self.restore_btn.setEnabled(False)
        self.embed_btn.setEnabled(bool(self.sim_hwnd))
        self.connection.setText("●  WINDOW RESTORED")
        self.status.showMessage(
            "Simulator window restored to a separate Windows window."
        )

    def closeEvent(self, event):
        self.held_keys.clear()
        if self.viewport.hosted_hwnd:
            self.restore_window()
        # Leave the simulator running if the user closes the dashboard.
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = LifeMAS()
    window.show()
    sys.exit(app.exec())
