from pathlib import Path

p = Path("ui/main.py")
s = p.read_text(encoding="utf-8")

if "import airsim" in s:
    raise SystemExit("AirSim import already exists. Stop to avoid duplicate edits.")

replacements = [
    (
        "import sys\nfrom ctypes import wintypes",
        "import sys\nimport airsim\nfrom ctypes import wintypes",
        "import"
    ),
    (
        "        self.process = None\n",
        "        self.process = None\n        self.api_client = None\n",
        "API client"
    ),
    (
        '        self.restore_btn.setEnabled(False)\n        controls.addWidget(self.launch_btn)',
        '        self.restore_btn.setEnabled(False)\n'
        '        self.connect_api_btn = QPushButton("3   Connect API")\n'
        '        self.connect_api_btn.clicked.connect(self.connect_api)\n'
        '        controls.addWidget(self.launch_btn)',
        "button"
    ),
]

for old, new, name in replacements:
    count = s.count(old)
    if count != 1:
        raise SystemExit(f"Patch stopped: '{name}' matched {count} times.")

# Do not write until every marker has been verified.
for old, new, name in replacements:
    s = s.replace(old, new, 1)

marker = "    def launch_simulator(self):\n"
if s.count(marker) != 1:
    raise SystemExit("Could not uniquely locate launch_simulator.")

methods = '''    def connect_api(self):
        try:
            client = airsim.MultirotorClient(
                ip="127.0.0.1", port=41451, timeout_value=5
            )
            client.confirmConnection()
            client.getMultirotorState()

            self.api_client = client
            self.connect_api_btn.setEnabled(False)
            self.telemetry_timer.start()
            self.connection.setText("●  AIRSIM API CONNECTED")
            self.connection.setStyleSheet("color: #72e0c0;")
            self.status.showMessage("Connected. Read-only telemetry active.")
            self.update_telemetry()
        except Exception as exc:
            QMessageBox.warning(self, "API connection failed", str(exc))

    def update_telemetry(self):
        if self.api_client is None:
            return
        try:
            state = self.api_client.getMultirotorState()
            pos = state.kinematics_estimated.position
            vel = state.kinematics_estimated.linear_velocity

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
            self.status.showMessage(f"Telemetry error: {exc}")

'''

s = s.replace(marker, methods + marker, 1)

timer_marker = "        self.poll_timer.timeout.connect(self.check_simulator_window)\n"
if s.count(timer_marker) != 1:
    raise SystemExit("Could not uniquely locate simulator timer.")

s = s.replace(
    timer_marker,
    timer_marker +
    "\n        self.telemetry_timer = QTimer(self)\n"
    "        self.telemetry_timer.setInterval(500)\n"
    "        self.telemetry_timer.timeout.connect(self.update_telemetry)\n",
    1
)

# Add a visible button to the existing controls row.
controls_marker = "        controls.addWidget(self.restore_btn)\n"
if s.count(controls_marker) != 1:
    raise SystemExit("Could not uniquely locate controls row.")
s = s.replace(
    controls_marker,
    controls_marker + "        controls.addWidget(self.connect_api_btn)\n",
    1
)

backup = Path("ui/main.py.backup")
if not backup.exists():
    raise SystemExit("Backup missing; refusing to write.")

p.write_text(s, encoding="utf-8")
print("Dashboard updated. Backup remains at ui/main.py.backup")
