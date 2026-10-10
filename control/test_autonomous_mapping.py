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

def test_autonomous_mapping_flight():
    print("=== Testing Real Autonomous Mapping Flight with AirSimNH ===")
    
    # Launch AirSimNH if not running
    process = None
    try:
        client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=2)
        client.confirmConnection()
        print("[INFO] Connected to running AirSim process.")
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
        assert connected, "Failed to connect to AirSim RPC server."

    vehicles = client.listVehicles()
    print(f"\n[VEHICLES LIST] Active vehicles: {vehicles}")
    assert "Drone1" in vehicles
    vehicle = "Drone1"

    # Enable API Control & Arm
    print(f"[MISSION] Enabling API Control for {vehicle}...")
    client.enableApiControl(True, vehicle_name=vehicle)
    
    print(f"[MISSION] Arming {vehicle}...")
    armed = client.armDisarm(True, vehicle_name=vehicle)
    print(f"[MISSION] Armed status: {armed}")
    assert armed

    # Takeoff
    print(f"[MISSION] Taking off to 3.0m altitude...")
    client.takeoffAsync(timeout_sec=15, vehicle_name=vehicle).join()
    time.sleep(1.0)
    
    state = client.getMultirotorState(vehicle_name=vehicle)
    alt = -state.kinematics_estimated.position.z_val
    print(f"[TELEMETRY] Takeoff complete. Current altitude: {alt:.2f} meters")

    # Flight & Real-time LiDAR Accumulation Loop
    print("[MISSION] Starting forward flight (v=1.5 m/s, duration=3.0s) with continuous LiDAR mapping...")
    
    flight_future = client.moveByVelocityZAsync(
        1.5, 0.0, -3.0, 3.0,
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=airsim.YawMode(False, 0),
        vehicle_name=vehicle
    )

    accumulated_points = np.empty((0, 3), dtype=np.float32)
    scans_received = 0

    start_time = time.time()
    while time.time() - start_time < 3.5:
        lidar_data = client.getLidarData(lidar_name="Lidar1", vehicle_name=vehicle)
        curr_state = client.getMultirotorState(vehicle_name=vehicle)
        pos = curr_state.kinematics_estimated.position
        
        if lidar_data and len(lidar_data.point_cloud) >= 3:
            pts_raw = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
            valid_mask = np.isfinite(pts_raw).all(axis=1)
            pts_valid = pts_raw[valid_mask]
            
            if len(pts_valid) > 0:
                pts_vis = transform_sensor_to_world_vis(pts_valid, lidar_data.pose, curr_state.kinematics_estimated)
                if len(accumulated_points) == 0:
                    accumulated_points = pts_vis
                else:
                    accumulated_points = np.vstack((accumulated_points, pts_vis))
                scans_received += 1
                print(f"  Scan #{scans_received:02d} @ Drone Pos (X={pos.x_val:.2f}, Y={pos.y_val:.2f}, Z={-pos.z_val:.2f}m) -> +{len(pts_vis):,} pts -> Total Map: {len(accumulated_points):,} pts")
        
        time.sleep(0.15)

    flight_future.join()
    print(f"[MAPPING RESULT] Total LiDAR scans recorded: {scans_received}")
    print(f"[MAPPING RESULT] Total accumulated 3D map points: {len(accumulated_points):,} points")
    assert scans_received > 0
    assert len(accumulated_points) > 5000

    # Hover & Land
    print("[MISSION] Hovering briefly...")
    client.hoverAsync(vehicle_name=vehicle).join()
    time.sleep(1.0)

    print("[MISSION] Landing drone...")
    client.landAsync(timeout_sec=30, vehicle_name=vehicle).join()

    land_state = client.getMultirotorState(vehicle_name=vehicle).landed_state
    print(f"[MISSION] Landed state check: {land_state}")
    
    client.armDisarm(False, vehicle_name=vehicle)
    client.enableApiControl(False, vehicle_name=vehicle)
    print("[MISSION COMPLETE] Drone safely landed and disarmed!")

if __name__ == "__main__":
    test_autonomous_mapping_flight()
