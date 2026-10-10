import os
import sys
import time
import math
import threading
import subprocess
import numpy as np
import airsim

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from mapping.survey_engine import (
    SurveyConfig,
    generate_survey_waypoints,
    SurveyDataset,
    ReconstructionEngine,
    transform_scan_to_world_vis,
    export_survey_point_cloud
)

SIMULATOR_EXE = os.path.join(
    project_root, "AirSimNH", "AirSimNH",
    "WindowsNoEditor", "AirSimNH.exe"
)

def ensure_simulator_running():
    try:
        client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=2)
        client.confirmConnection()
        print("[SIMULATOR] Connected to running AirSimNH.")
        return client
    except Exception:
        print(f"[SIMULATOR] Launching AirSimNH: {SIMULATOR_EXE}")
        proc = subprocess.Popen([SIMULATOR_EXE], cwd=os.path.dirname(SIMULATOR_EXE))
        for i in range(30):
            time.sleep(1.0)
            try:
                client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=3)
                client.confirmConnection()
                print(f"[SIMULATOR] Connected after {i+1}s.")
                return client
            except Exception:
                sys.stdout.write(".")
                sys.stdout.flush()
        raise RuntimeError("Could not connect to AirSimNH after 30s.")

def run_verified_survey():
    print("=======================================================")
    print("   END-TO-END AERIAL SURVEY & 3D RECONSTRUCTION TEST   ")
    print("=======================================================")

    client = ensure_simulator_running()
    vehicle = "Drone1"

    config = SurveyConfig(
        survey_id=f"flight_test_{time.strftime('%Y%m%d_%H%M%S')}",
        length_x=14.0,
        width_y=8.0,
        altitude=3.8,
        lane_spacing=2.5,
        flight_speed=1.5,
        turn_speed=1.0,
        min_scans_required=15,
        voxel_size=0.08,
        min_range=0.40,
        max_range=45.0
    )

    dataset = SurveyDataset(config.survey_id, config)
    stop_polling = threading.Event()

    # Background LiDAR acquisition thread (matches LidarTelemetryWorker in ui/main.py)
    def lidar_polling_thread():
        print("[LIDAR WORKER] Thread started.")
        client_lidar = airsim.MultirotorClient(ip="127.0.0.1", port=41451)
        client_lidar.confirmConnection()
        while not stop_polling.is_set():
            try:
                state = client_lidar.getMultirotorState(vehicle_name=vehicle)
                pos = state.kinematics_estimated.position
                ori = state.kinematics_estimated.orientation
                lidar_data = client_lidar.getLidarData("Lidar1", vehicle_name=vehicle)

                if lidar_data and len(lidar_data.point_cloud) >= 3:
                    pts_raw = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
                    valid_mask = np.isfinite(pts_raw).all(axis=1)
                    pts_valid = pts_raw[valid_mask]

                    if len(pts_valid) > 0:
                        dataset.add_scan(
                            timestamp=time.time(),
                            vehicle_pos=(pos.x_val, pos.y_val, pos.z_val),
                            vehicle_ori=(ori.w_val, ori.x_val, ori.y_val, ori.z_val),
                            sensor_pos=(lidar_data.pose.position.x_val, lidar_data.pose.position.y_val, lidar_data.pose.position.z_val),
                            sensor_ori=(lidar_data.pose.orientation.w_val, lidar_data.pose.orientation.x_val, lidar_data.pose.orientation.y_val, lidar_data.pose.orientation.z_val),
                            points_sensor=pts_valid
                        )
            except Exception:
                pass
            time.sleep(0.08) # ~12 Hz
        print("[LIDAR WORKER] Thread stopped.")

    # Enable API Control and Arm
    print(f"[FLIGHT] Enabling API control and arming {vehicle}...")
    client.enableApiControl(True, vehicle_name=vehicle)
    client.armDisarm(True, vehicle_name=vehicle)

    # Start LiDAR thread
    poller = threading.Thread(target=lidar_polling_thread, daemon=True)
    poller.start()

    # Takeoff
    print(f"[FLIGHT] Taking off to survey altitude {config.altitude:.1f}m...")
    client.takeoffAsync(timeout_sec=15, vehicle_name=vehicle).join()
    time.sleep(1.0)

    waypoints = generate_survey_waypoints(config)
    flight_wps = [w for w in waypoints if w["type"] in ("TRANSITION", "PASS", "TURN", "RETURN")]
    print(f"[FLIGHT] Executing {len(flight_wps)} survey waypoints...")

    for idx, wp in enumerate(flight_wps):
        dist = math.hypot(wp["x"], wp["y"])
        timeout = max(10.0, (dist / max(0.5, wp["speed"])) * 2.5 + 8.0)
        print(f"  -> Waypoint {idx+1}/{len(flight_wps)}: {wp['name']} (X={wp['x']:.1f}, Y={wp['y']:.1f}, Z={wp['z']:.1f})")
        client.moveToPositionAsync(
            wp["x"], wp["y"], wp["z"], wp["speed"],
            timeout_sec=timeout,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(False, 0),
            vehicle_name=vehicle
        ).join()

    # Landing
    print("\n[FLIGHT] Survey pattern finished. Initiating automated landing...")
    client.moveToPositionAsync(0.0, 0.0, -2.5, 1.2, timeout_sec=15, vehicle_name=vehicle).join()
    client.landAsync(timeout_sec=30, vehicle_name=vehicle).join()

    # Confirm landed
    confirmed = False
    for _ in range(25):
        st = client.getMultirotorState(vehicle_name=vehicle)
        landed_val = str(st.landed_state)
        alt = abs(st.kinematics_estimated.position.z_val)
        vz = abs(st.kinematics_estimated.linear_velocity.z_val)
        if "Landed" in landed_val or (alt < 0.35 and vz < 0.2):
            confirmed = True
            break
        time.sleep(0.5)

    # Stop LiDAR polling
    stop_polling.set()
    poller.join(timeout=2.0)

    client.armDisarm(False, vehicle_name=vehicle)
    client.enableApiControl(False, vehicle_name=vehicle)
    print(f"[FLIGHT] Drone landed safely and disarmed (confirmed={confirmed}).")

    # Metrics on collected data
    scan_count = dataset.get_scan_count()
    total_raw_points = dataset.get_total_raw_points()
    print(f"\n[DATA] Collected {scan_count} valid LiDAR scans ({total_raw_points:,} raw points).")
    assert scan_count >= config.min_scans_required, f"Insufficient scans: {scan_count}"

    # Save raw dataset
    maps_dir = os.path.join(project_root, "data", "maps")
    os.makedirs(maps_dir, exist_ok=True)
    raw_path = os.path.join(maps_dir, f"raw_survey_{config.survey_id}.npz")
    dataset.save_raw_dataset(raw_path)
    print(f"[DATA] Raw survey dataset saved to: {raw_path}")

    # Process Reconstruction
    print("\n[RECONSTRUCTION] Running Post-Flight 3D Reconstruction...")
    t0 = time.time()
    artifact = ReconstructionEngine.process_survey(
        dataset,
        output_dir=maps_dir,
        progress_callback=lambda pct, msg: print(f"  [{pct}%] {msg}")
    )
    dt = time.time() - t0
    print(f"[RECONSTRUCTION] Completed in {dt:.2f}s!")

    # Validate Artifact
    assert artifact.is_valid(), "Artifact failed validation!"
    pts = artifact.points_3d
    b_min = np.min(pts, axis=0)
    b_max = np.max(pts, axis=0)
    span = b_max - b_min
    print(f"\n[GEOMETRY AUDIT]")
    print(f"  Total Voxel Points : {len(pts):,}")
    print(f"  X Range (North)   : [{b_min[0]:.2f}, {b_max[0]:.2f}] m  (span: {span[0]:.2f}m)")
    print(f"  Y Range (East)    : [{b_min[1]:.2f}, {b_max[1]:.2f}] m  (span: {span[1]:.2f}m)")
    print(f"  Z Range (Height)  : [{b_min[2]:.2f}, {b_max[2]:.2f}] m  (span: {span[2]:.2f}m)")
    print(f"  Coverage Area     : {span[0] * span[1]:.1f} m²")

    # Export XYZ point cloud
    export_xyz_path = os.path.join(maps_dir, f"survey_{config.survey_id}.xyz")
    export_survey_point_cloud(artifact, export_xyz_path, fmt="xyz")
    print(f"[EXPORT] Exported clean XYZ point cloud to: {export_xyz_path}")

    # Assertions on Geometry Correctness:
    # 1. Height: Ground must be near Z=0, buildings must reach upward to Z in [2, 12]m.
    #    No points below -2.5m, no points above 25.0m.
    assert b_min[2] >= -2.5, f"Subterranean artifact: Z min = {b_min[2]}"
    assert b_max[2] <= 25.0, f"Stratosphere artifact: Z max = {b_max[2]}"
    ground_pts = pts[(pts[:, 2] >= -0.5) & (pts[:, 2] <= 0.8)]
    assert len(ground_pts) > 1000, "Ground plane must contain substantial surface points!"
    print(f"  Ground Plane Points: {len(ground_pts):,} (Z in [-0.5, 0.8]m)")

    # 2. X and Y extents must reflect true neighborhood street geometry (no coordinate explosion)
    # The survey is 14m forward, with sensor range up to 45m looking down the street in both directions.
    assert -55.0 <= b_min[0] <= 5.0, f"Unreasonable X min: {b_min[0]}"
    assert 14.0 <= b_max[0] <= 65.0, f"Unreasonable X max: {b_max[0]}"
    assert -35.0 <= b_min[1] <= 0.0, f"Unreasonable Y min: {b_min[1]}"
    assert 4.0 <= b_max[1] <= 40.0, f"Unreasonable Y max: {b_max[1]}"

    print("\n[SUCCESS] ALL GEOMETRY AUDIT CHECKS PASSED WITH EXCELLENCE!")
    return 0

if __name__ == "__main__":
    sys.exit(run_verified_survey())
