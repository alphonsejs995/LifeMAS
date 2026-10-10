import os
import sys
import time
import subprocess
import airsim
import numpy as np

PROJECT_ROOT = "C:\\Users\\alpho\\Documents\\LifeMAS"
SIMULATOR_EXE = os.path.join(
    PROJECT_ROOT, "AirSimNH", "AirSimNH",
    "WindowsNoEditor", "AirSimNH.exe"
)

def run_diagnostic():
    print("=== AirSimNH Single-Drone LiDAR Diagnostic ===")
    
    # 1. Launch AirSimNH if not running
    process = None
    try:
        client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=2)
        client.confirmConnection()
        print("[INFO] Connected to existing AirSim process.")
    except Exception:
        print(f"[INFO] Launching AirSimNH from: {SIMULATOR_EXE}")
        process = subprocess.Popen([SIMULATOR_EXE], cwd=os.path.dirname(SIMULATOR_EXE))
        print("[INFO] Waiting for AirSimNH RPC server to start...")
        connected = False
        for i in range(30):
            time.sleep(1.0)
            try:
                client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=2)
                client.confirmConnection()
                connected = True
                print(f"[INFO] Connected to AirSim API after {i+1} seconds!")
                break
            except Exception:
                sys.stdout.write(".")
                sys.stdout.flush()
        if not connected:
            print("\n[ERROR] Failed to connect to AirSimNH API within 30 seconds.")
            return

    # 2. Query Vehicles
    vehicles = client.listVehicles()
    print(f"\n[VEHICLES LIST] Active vehicles reported by AirSim: {vehicles}")
    if not vehicles:
        print("[ERROR] No vehicles found.")
        return
    
    vehicle = vehicles[0]
    print(f"[ACTIVE VEHICLE] Selected vehicle: '{vehicle}'")

    # 3. Query State
    state = client.getMultirotorState(vehicle_name=vehicle)
    pos = state.kinematics_estimated.position
    ori = state.kinematics_estimated.orientation
    print(f"[STATE] Kinematics Position: X={pos.x_val:.3f}, Y={pos.y_val:.3f}, Z={pos.z_val:.3f}")
    print(f"[STATE] Kinematics Orientation: W={ori.w_val:.3f}, X={ori.x_val:.3f}, Y={ori.y_val:.3f}, Z={ori.z_val:.3f}")
    print(f"[STATE] Ready={state.ready}, CanArm={state.can_arm}, LandedState={state.landed_state}")

    # 4. Query Lidar1 Data
    lidar_data = client.getLidarData(lidar_name="Lidar1", vehicle_name=vehicle)
    print(f"\n[LIDAR QUERY] Time Stamp: {lidar_data.time_stamp}")
    print(f"[LIDAR QUERY] Raw Point Cloud Length (floats): {len(lidar_data.point_cloud)}")
    
    if len(lidar_data.point_cloud) >= 3:
        pts = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
        num_points = len(pts)
        is_finite = np.isfinite(pts).all()
        print(f"[LIDAR VERIFIED] 3D Point Count: {num_points:,} points")
        print(f"[LIDAR VERIFIED] All float values finite: {is_finite}")
        print(f"[LIDAR VERIFIED] Sample Point 0 (Sensor Frame): {pts[0]}")
        print(f"[LIDAR VERIFIED] Sensor Pose Position: X={lidar_data.pose.position.x_val:.3f}, Y={lidar_data.pose.position.y_val:.3f}, Z={lidar_data.pose.position.z_val:.3f}")
        print(f"[LIDAR VERIFIED] Sensor Pose Orientation: W={lidar_data.pose.orientation.w_val:.3f}, X={lidar_data.pose.orientation.x_val:.3f}, Y={lidar_data.pose.orientation.y_val:.3f}, Z={lidar_data.pose.orientation.z_val:.3f}")
    else:
        print("[LIDAR WARNING] Lidar1 returned 0 points.")

    print("\n=== Diagnostic Completed Successfully ===")

if __name__ == "__main__":
    run_diagnostic()
