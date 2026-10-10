import os
import sys
import time
import subprocess
import airsim
import numpy as np

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from ui.main import transform_sensor_to_world_vis

SIMULATOR_EXE = os.path.join(
    project_root, "AirSimNH", "AirSimNH",
    "WindowsNoEditor", "AirSimNH.exe"
)

def inspect_lidar_geometry():
    print("=== AirSim LiDAR Geometry & Ground-Height Diagnostic ===")
    
    # 1. Connect or Launch AirSim
    process = None
    try:
        client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=2)
        client.confirmConnection()
        print("[CONNECT] Connected to existing AirSim process.")
    except Exception:
        print(f"[LAUNCH] Launching AirSimNH: {SIMULATOR_EXE}")
        process = subprocess.Popen([SIMULATOR_EXE], cwd=os.path.dirname(SIMULATOR_EXE))
        connected = False
        for i in range(25):
            time.sleep(1.0)
            try:
                client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=3)
                client.confirmConnection()
                connected = True
                print(f"[CONNECT] Connected to AirSim API after {i+1} seconds!")
                break
            except Exception:
                sys.stdout.write(".")
                sys.stdout.flush()
        if not connected:
            print("\n[ERROR] Connection failed.")
            return

    vehicles = client.listVehicles()
    print("[VEHICLES]", vehicles)
    if not vehicles:
        return
    vehicle = vehicles[0]

    state = client.getMultirotorState(vehicle_name=vehicle)
    pos = state.kinematics_estimated.position
    ori = state.kinematics_estimated.orientation
    print(f"[STATE] Vehicle: {vehicle}, Pos NED: X={pos.x_val:.3f}, Y={pos.y_val:.3f}, Z={pos.z_val:.3f}")
    print(f"[STATE] Altitude: {-pos.z_val:.3f} m")

    lidar_data = client.getLidarData(lidar_name="Lidar1", vehicle_name=vehicle)
    print(f"[LIDAR] Raw point cloud float count: {len(lidar_data.point_cloud)}")
    print(f"[LIDAR] Sensor Pose Position: {lidar_data.pose.position}")
    print(f"[LIDAR] Sensor Pose Orientation: {lidar_data.pose.orientation}")

    if len(lidar_data.point_cloud) >= 3:
        pts_raw = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
        print(f"[LIDAR] 3D Points: {len(pts_raw)}")
        print(f"[LIDAR] Raw Sensor Frame X min/max: [{np.min(pts_raw[:, 0]):.2f}, {np.max(pts_raw[:, 0]):.2f}]")
        print(f"[LIDAR] Raw Sensor Frame Y min/max: [{np.min(pts_raw[:, 1]):.2f}, {np.max(pts_raw[:, 1]):.2f}]")
        print(f"[LIDAR] Raw Sensor Frame Z min/max: [{np.min(pts_raw[:, 2]):.2f}, {np.max(pts_raw[:, 2]):.2f}]")

        pts_vis = transform_sensor_to_world_vis(pts_raw, lidar_data.pose, state.kinematics_estimated)
        print(f"[VISUAL WORLD] X min/max: [{np.min(pts_vis[:, 0]):.2f}, {np.max(pts_vis[:, 0]):.2f}]")
        print(f"[VISUAL WORLD] Y min/max: [{np.min(pts_vis[:, 1]):.2f}, {np.max(pts_vis[:, 1]):.2f}]")
        print(f"[VISUAL WORLD] Z min/max: [{np.min(pts_vis[:, 2]):.2f}, {np.max(pts_vis[:, 2]):.2f}]")

        # Ground analysis
        z_vals = pts_vis[:, 2]
        ground_hits = np.sum(z_vals <= 0.25)
        obstacle_hits = np.sum((z_vals > 0.25) & (z_vals < 3.0))
        high_hits = np.sum(z_vals >= 3.0)
        print(f"\n[GEOMETRY DISTRIBUTION]")
        print(f"  Ground returns (Z <= 0.25m): {ground_hits} ({ground_hits/len(z_vals)*100:.1f}%)")
        print(f"  Obstacle returns (0.25m < Z < 3.0m): {obstacle_hits} ({obstacle_hits/len(z_vals)*100:.1f}%)")
        print(f"  High/Sky returns (Z >= 3.0m): {high_hits} ({high_hits/len(z_vals)*100:.1f}%)")

if __name__ == "__main__":
    inspect_lidar_geometry()
