from pathlib import Path

path = Path("ui/main.py")
source = path.read_text(encoding="utf-8")

if "import airsim" in source:
    raise SystemExit("AirSim integration already exists. No changes made.")

backup = Path("ui/main.py.backup")
backup.write_text(source, encoding="utf-8")
print("Backup created:", backup)

def replace_once(old, new, description):
    global source
    count = source.count(old)
    if count != 1:
        raise SystemExit(
            f"Could not safely patch '{description}': "
            f"expected one matching section, found {count}. "
            "Original file remains unchanged."
        )
    source = source.replace(old, new, 1)

replace_once(
    "import sys\nfrom ctypes import wintypes",
    "import sys\nimport airsim\nfrom ctypes import wintypes",
    "AirSim import"
)

replace_once(
    '        self.wait_count = 0\n',
    '''        self.wait_count = 0
        self.api_client = None
''',
    "API client state"
)

replace_once(
    '''        self.restore_btn.setEnabled(False)
        controls.addWidget(self.launch_btn)
        controls.addWidget(self.embed_btn)
        controls.addWidget(self.restore_btn)
        left.addLayout(controls)
''',
    '''        self.restore_btn.setEnabled(False)
        self.connect_api_btn = QPushButton("3   Connect API")
        self.connect_api_btn.clicked.connect(self.connect_api)
        controls.addWidget(self.launch_btn)
        controls.addWidget(self.embed_btn)
        controls.addWidget(self.restore_btn)
        controls.addWidget(self.connect_api_btn)
        left.addLayout(controls)
''',
    "API connection button"
)

replace_once(
    '''        self.poll_timer.timeout.connect(self.check_simulator_window)
''',
    '''        self.poll_timer.timeout.connect(self.check_simulator_window)

        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.setInterval(500)
        self.telemetry_timer.timeout.connect(self.update_telemetry)
''',
    "telemetry timer"
)

replace_once(
    '''    def launch_simulator(self):
''',
    '''    def connect_api(self):
        if self.api_client is not None:
            self.status.showMessage("AirSim API is already connected.")
            return

        self.connect_api_btn.setEnabled(False)
        self.status.showMessage("Connecting to AirSimNH API...")

        try:
            client = airsim.MultirotorClient(
                ip="127.0.0.1",
                port=41451,
                timeout_value=5
            )
            client.confirmConnection()

            # Read-only connection for this stage:
            # do not arm, take off, or send movement commands.
            state = client.getMultirotorState()

            self.api_client = client
            self.telemetry_timer.start()
            self.connection.setText("●  AIRSIM API CONNECTED")
            self.connection.setStyleSheet("color: #72e0c0;")
            self.sim_state.setText("●  API connected · telemetry active")
            self.status.showMessage(
                "Connected to AirSimNH. Read-only telemetry is active."
            )
            self.update_telemetry()

        except Exception as exc:
            self.api_client = None
            self.connect_api_btn.setEnabled(True)
            QMessageBox.warning(
                self,
                "AirSim API connection failed",
                f"Could not connect to AirSimNH on port 41451.\\n\\n{exc}"
            )
            self.status.showMessage("API connection failed.")

    def update_telemetry(self):
        if self.api_client is None:
            return

        try:
            state = self.api_client.getMultirotorState()
            pos = state.kinematics_estimated.position
            vel = state.kinematics_estimated.linear_velocity

            # AirSim uses NED coordinates: positive Z points downward.
            self.telemetry["Active drones"].setText("1")
            self.telemetry["Altitude"].setText(f"{-pos.z_val:.2f} m*")
            self.telemetry["Position"].setText(
                f"X {pos.x_val:.1f}  Y {pos.y_val:.1f}"
            )
            self.telemetry["Flight mode"].setText(
                f"V {vel.x_val:.1f}, {vel.y_val:.1f}, {vel.z_val:.1f}"
            )

        except Exception as exc:
            self.telemetry_timer.stop()
            self.connection.setText("●  API CONNECTION LOST")
            self.connection.setStyleSheet("color: #f0bd72;")
            self.sim_state.setText("●  Telemetry unavailable")
            self.status.showMessage(f"Telemetry stopped: {exc}")
            self.connect_api_btn.setEnabled(True)
            self.api_client = None

    def launch_simulator(self):
''',
    "telemetry methods"
)

replace_once(
    '''        # Leave the simulator running if the user closes the dashboard.
        event.accept()
''',
    '''        self.telemetry_timer.stop()
        # Leave the simulator running if the user closes the dashboard.
        event.accept()
''',
    "close cleanup"
)

path.write_text(source, encoding="utf-8")
print("Dashboard patched successfully.")
print("Backup saved at:", backup)
