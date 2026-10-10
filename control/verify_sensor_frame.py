import os
import sys
import time
import subprocess
import math
import numpy as np
import airsim

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIMULATOR_EXE = os.path.join(
    project_root, "AirSimNH", "AirSimNH",
    "WindowsNoEditor", "AirSimNH.exe"
)

def main():
    print("[TEST] Checking AirSim connection...")
    client = None
    try:
        client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=2)
        client.confirmConnection()
        print("[CONNECT] Connected to existing AirSim process.")
    except Exception:
        print(f"[LAUNCH] Starting AirSimNH: {SIMULATOR_EXE}")
        proc = subprocess.Popen([SIMULATOR_EXE], cwd=os.path.dirname(SIMULATOR_EXE))
        connected = False
        for i in range(30):
            time.sleep(1.0)
            try:
                client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=3)
                client.confirmConnection()
                connected = True
                print(f"[CONNECT] Connected to AirSim after {i+1}s!")
                break
            except Exception:
                sys.stdout.write(".")
                sys.stdout.flush()
        if not connected:
            print("\n[FAIL] Could not connect to AirSim.")
            return 1

    client.enableApiControl(True, "Drone1")
    client.armDisarm(True, "Drone1")

    print("\n[TEST] Taking off to 3.0m...")
    client.takeoffAsync(timeout_sec=5.0, vehicle_name="Drone1").join()
    client.moveToPositionAsync(0, 0, -3.0, 2.0, vehicle_name="Drone1").join()
    time.sleep(1.0)

    # Hover at (0, 0, -3.0) and inspect LiDAR data
    lidar_data = client.getLidarData("Lidar1", "Drone1")
    pts_raw = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
    state = client.getMultirotorState(vehicle_name="Drone1")
    pos0 = state.kinematics_estimated.position
    print(f"\n--- AT HOVER (X={pos0.x_val:.2f}, Y={pos0.y_val:.2f}, Z={pos0.z_val:.2f}) ---")
    print(f"LiDAR data pose pos: ({lidar_data.pose.position.x_val:.2f}, {lidar_data.pose.position.y_val:.2f}, {lidar_data.pose.position.z_val:.2f})")
    print(f"Raw points count: {len(pts_raw)}")
    if len(pts_raw) > 0:
        print(f"Raw points X range: [{pts_raw[:,0].min():.2f}, {pts_raw[:,0].max():.2f}]")
        print(f"Raw points Y range: [{pts_raw[:,1].min():.2f}, {pts_raw[:,1].max():.2f}]")
        print(f"Raw points Z range: [{pts_raw[:,2].min():.2f}, {pts_raw[:,2].max():.2f}]")
        ranges = np.linalg.norm(pts_raw, axis=1)
        print(f"Sensor ranges: min={ranges.min():.2f}m, mean={ranges.mean():.2f}m, max={ranges.max():.2f}m")

    # Move to X=6.0, Y=2.0, Z=-3.0
    print("\n[TEST] Moving to X=6.0m, Y=2.0m, Z=-3.0m...")
    client.moveToPositionAsync(6.0, 2.0, -3.0, 2.0, vehicle_name="Drone1").join()
    time.sleep(1.0)

    lidar_data = client.getLidarData("Lidar1", "Drone1")
    pts_raw = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
    state = client.getMultirotorState(vehicle_name="Drone1")
    pos1 = state.kinematics_estimated.position
    print(f"\n--- AT OFFSET (X={pos1.x_val:.2f}, Y={pos1.y_val:.2f}, Z={pos1.z_val:.2f}) ---")
    print(f"LiDAR data pose pos: ({lidar_data.pose.position.x_val:.2f}, {lidar_data.pose.position.y_val:.2f}, {lidar_data.pose.position.z_val:.2f})")
    print(f"Raw points count: {len(pts_raw)}")
    if len(pts_raw) > 0:
        print(f"Raw points X range: [{pts_raw[:,0].min():.2f}, {pts_raw[:,0].max():.2f}]")
        print(f"Raw points Y range: [{pts_raw[:,1].min():.2f}, {pts_raw[:,1].max():.2f}]")
        print(f"Raw points Z range: [{pts_raw[:,2].min():.2f}, {pts_raw[:,2].max():.2f}]")
        ranges = np.linalg.norm(pts_raw, axis=1)
        print(f"Sensor ranges: min={ranges.min():.2f}m, mean={ranges.mean():.2f}m, max={ranges.max():.2f}m")

    # Land
    print("\n[TEST] Returning home and landing...")
    client.moveToPositionAsync(0, 0, -3.0, 2.0, vehicle_name="Drone1").join()
    client.landAsync(vehicle_name="Drone1").join()
    client.armDisarm(False, "Drone1")
    client.enableApiControl(False, "Drone1")
    print("[TEST COMPLETE]")
    return 0

if __name__ == "__main__":
    sys.exit(main())
