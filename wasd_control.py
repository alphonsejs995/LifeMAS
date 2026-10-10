import sys
import time
import airsim
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout

HOST = "127.0.0.1"
PORT = 41451
DRONE = "Drone1"
SPEED = 3.0

class DroneController(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LifeMAS WASD Controller")
        self.resize(420, 180)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        layout = QVBoxLayout(self)
        self.status = QLabel("Connecting to AirSimNH...")
        self.status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.status)

        self.keys = set()
        self.client = airsim.MultirotorClient(
            ip=HOST, port=PORT, timeout_value=10
        )
        self.client.confirmConnection()
        self.client.enableApiControl(True, vehicle_name=DRONE)

        self.status.setText(
            "Connected — click this window, then hold WASD.\n"
            "W/S forward/back • A/D left/right\n"
            "R/F up/down • Q/E turn • Space stop"
        )

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_motion)
        self.timer.start(50)

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() == Qt.Key.Key_Escape:
            self.close()
            return
        self.keys.add(event.key())

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            self.keys.discard(event.key())

    def update_motion(self):
        if Qt.Key.Key_Space in self.keys:
            vx = vy = vz = yaw = 0.0
        else:
            vx = SPEED * (
                (Qt.Key.Key_W in self.keys) -
                (Qt.Key.Key_S in self.keys)
            )
            vy = SPEED * (
                (Qt.Key.Key_D in self.keys) -
                (Qt.Key.Key_A in self.keys)
            )
            vz = SPEED * (
                (Qt.Key.Key_F in self.keys) -
                (Qt.Key.Key_R in self.keys)
            )
            yaw = 30.0 * (
                (Qt.Key.Key_E in self.keys) -
                (Qt.Key.Key_Q in self.keys)
            )

        try:
            self.client.moveByVelocityAsync(
                float(vx), float(vy), float(vz), 0.15,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(True, float(yaw)),
                vehicle_name=DRONE
            )
        except Exception as exc:
            self.status.setText(f"AirSim command error: {exc}")

    def closeEvent(self, event):
        try:
            self.client.moveByVelocityAsync(
                0, 0, 0, 0.2, vehicle_name=DRONE
            )
            self.client.enableApiControl(False, vehicle_name=DRONE)
        except Exception:
            pass
        event.accept()

app = QApplication(sys.argv)
window = DroneController()
window.show()
window.activateWindow()
window.setFocus()
sys.exit(app.exec())
