import sys
import os
import numpy as np

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from PySide6.QtWidgets import QApplication
from ui.main import (
    LifeMAS, SpatialMapWindow, LidarTelemetryWorker,
    transform_sensor_to_world_vis, compute_point_colors
)

class DummyPose:
    def __init__(self, x=0, y=0, z=0, w=1, qx=0, qy=0, qz=0):
        class Pos: pass
        class Ori: pass
        self.position = Pos()
        self.position.x_val = x
        self.position.y_val = y
        self.position.z_val = z
        self.orientation = Ori()
        self.orientation.w_val = w
        self.orientation.x_val = qx
        self.orientation.y_val = qy
        self.orientation.z_val = qz

class DummyKinematics:
    def __init__(self, x=0, y=0, z=-10, w=1, qx=0, qy=0, qz=0):
        self.position = DummyPose(x, y, z).position
        self.orientation = DummyPose(x, y, z, w, qx, qy, qz).orientation

def run_comprehensive_tests():
    print("=== 1. Testing Transform & Coordinate Math ===")
    pts_sensor = np.array([
        [1.0, 2.0, 3.0],
        [0.0, 0.0, 0.0],
        [5.0, -1.0, 0.5]
    ], dtype=np.float32)
    
    sensor_pose = DummyPose(0, 0, 0)
    drone_kinematics = DummyKinematics(0, 0, -10.0) # 10m altitude
    
    pts_vis = transform_sensor_to_world_vis(pts_sensor, sensor_pose, drone_kinematics)
    print("Transformed visual points:\n", pts_vis)
    assert len(pts_vis) == 3
    assert np.isclose(pts_vis[0, 2], 7.0)
    
    colors = compute_point_colors(pts_vis)
    assert colors.shape == (3, 4)
    print("[PASS] Transform & coordinate math verified.")

    print("\n=== 2. Testing UI Window & Maximized Map Window ===")
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    window = LifeMAS()
    window.show()
    app.processEvents()

    # Verify Map window
    window.open_map_window()
    app.processEvents()
    assert window.map_window.isVisible()
    print("[PASS] SpatialMapWindow opened successfully.")

    # Feed point cloud batch
    pts_batch = np.random.uniform(-15, 15, (5088, 3)).astype(np.float32)
    window.map_window.add_point_cloud(pts_batch, "Lidar1", 5088, 0.0, 0.0, 3.0)
    window.update_sidebar_map_stats(pts_batch, "Lidar1", 5088, 0.0, 0.0, 3.0)
    app.processEvents()

    assert len(window.map_window.accumulated_points) == 5088
    print(f"[PASS] Map point cloud batch updated: {len(window.map_window.accumulated_points):,} points")

    # Update Telemetry UI with single drone
    mock_telemetry = {
        "active_vehicles": ["Drone1"],
        "pos_x": 4.5,
        "pos_y": 2.1,
        "altitude": 3.0,
        "vel_x": 1.5,
        "vel_y": 0.0,
        "vel_z": 0.0,
        "ready": True,
        "can_arm": True,
        "landed_state": "LandedState.Landed",
        "vehicle": "Drone1"
    }
    window.update_telemetry_ui(mock_telemetry)
    app.processEvents()
    assert window.telemetry["Active Vehicle"].text() == "Drone1"
    print("[PASS] Telemetry UI reports real single-drone configuration.")

    # Test Mission Status Update
    window.handle_mission_status("TAKEOFF", "Initiating autonomous takeoff...")
    window.map_window.update_mission_status("TAKEOFF", "Taking off...")
    app.processEvents()
    assert "TAKEOFF" in window.mission_status_lbl.text()
    print("[PASS] Autonomous mission status handling verified.")

    # Test Map Reset & Close
    window.map_window.reset_map()
    assert len(window.map_window.accumulated_points) == 0
    window.map_window.hide()
    app.processEvents()
    print("[PASS] Map reset & hide verified.")

    window.close()
    app.processEvents()
    print("\n=== ALL COMPREHENSIVE UI & MATH TESTS PASSED ===")

if __name__ == "__main__":
    run_comprehensive_tests()
