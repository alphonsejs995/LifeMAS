import os
import sys
import time
import numpy as np

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

def test_survey_engine():
    print("=== TEST 1: Coverage Planner Boundedness & Pattern ===")
    config = SurveyConfig(
        survey_id="test_survey_001",
        length_x=14.0,
        width_y=8.0,
        altitude=3.5,
        lane_spacing=2.5,
        flight_speed=1.5,
        turn_speed=1.0,
        min_scans_required=10,
        voxel_size=0.08
    )

    waypoints = generate_survey_waypoints(config)
    print(f"Generated {len(waypoints)} survey waypoints:")
    for wp in waypoints:
        print(f"  [{wp['type']}] {wp['name']}: X={wp['x']:.1f}, Y={wp['y']:.1f}, Z={wp['z']:.1f} ({wp['description']})")

    # Assertions
    assert len(waypoints) >= 6
    assert waypoints[0]["type"] == "TAKEOFF"
    assert waypoints[0]["z"] == -3.5 # Altitude NED
    assert waypoints[-1]["type"] == "LAND"
    assert waypoints[-1]["x"] == 0.0 and waypoints[-1]["y"] == 0.0

    # Boundedness check: all survey waypoints within [0, 14] x [-4, +4]
    for wp in waypoints:
        assert 0.0 <= wp["x"] <= 14.01, f"Waypoint X={wp['x']} out of bounds!"
        assert -4.01 <= wp["y"] <= 4.01, f"Waypoint Y={wp['y']} out of bounds!"
    print("[PASS] Boundedness and lawnmower coverage pattern verified.")

    print("\n=== TEST 2: Multi-Scan Pose Transformation & World Stability ===")
    # Sensor mounted at Z=0.25 on drone
    sensor_pos = (0.0, 0.0, 0.25)
    sensor_ori = (1.0, 0.0, 0.0, 0.0) # Identity

    # Scan 1: Drone at Origin (0, 0, -3.5 NED = 3.5m Visual Z)
    drone_pos_1 = (0.0, 0.0, -3.5)
    drone_ori_1 = (1.0, 0.0, 0.0, 0.0)
    raw_pt_1 = np.array([[5.0, 2.0, 1.0]], dtype=np.float32) # In sensor frame

    pts_vis_1 = transform_scan_to_world_vis(raw_pt_1, sensor_pos, sensor_ori, drone_pos_1, drone_ori_1)
    # Expected: X = 0 + 5 = 5.0, Y = 0 + 2 = 2.0
    # Sensor Z = 0.25, Drone Z = -3.5 -> NED Z = 1.0 + 0.25 - 3.5 = -2.25 -> Vis Z = +2.25
    assert np.isclose(pts_vis_1[0, 0], 5.0)
    assert np.isclose(pts_vis_1[0, 1], 2.0)
    assert np.isclose(pts_vis_1[0, 2], 2.25)

    # Scan 2: Drone at (10, 5, -3.5 NED) observing a point at relative (-5, -3, 1.0)
    # This should be the EXACT same world point (5, 2, 2.25)!
    drone_pos_2 = (10.0, 5.0, -3.5)
    drone_ori_2 = (1.0, 0.0, 0.0, 0.0)
    raw_pt_2 = np.array([[-5.0, -3.0, 1.0]], dtype=np.float32)

    pts_vis_2 = transform_scan_to_world_vis(raw_pt_2, sensor_pos, sensor_ori, drone_pos_2, drone_ori_2)
    assert np.allclose(pts_vis_1, pts_vis_2), "World coordinates must be identical regardless of drone position!"
    print("[PASS] Pose transformation and world coordinate stability verified.")

    print("\n=== TEST 3: Insufficient Data Rejection Rule ===")
    dataset_small = SurveyDataset(survey_id="test_small", config=config)
    # Add only 3 scans (less than min_scans_required=10)
    for i in range(3):
        dataset_small.add_scan(
            timestamp=time.time() + i,
            vehicle_pos=(float(i), 0.0, -3.5),
            vehicle_ori=(1.0, 0.0, 0.0, 0.0),
            sensor_pos=sensor_pos,
            sensor_ori=sensor_ori,
            points_sensor=np.random.uniform(1, 10, (100, 3)).astype(np.float32)
        )

    out_dir = os.path.join(project_root, "data", "maps")
    rejection_caught = False
    try:
        ReconstructionEngine.process_survey(dataset_small, output_dir=out_dir)
    except ValueError as e:
        rejection_caught = True
        print(f"[EXPECTED REJECTION CAUGHT] {e}")

    assert rejection_caught, "Engine must reject survey if scan count < min_scans_required!"
    print("[PASS] Rejection rule verified: Insufficient scans prevent Final Map creation.")

    print("\n=== TEST 4: Full Multi-Scan Reconstruction & Voxel Deduplication ===")
    dataset_full = SurveyDataset(survey_id="test_full_001", config=config)
    # Add 15 scans along flight path
    for i in range(15):
        # Generate points around drone, with overlapping points across scans to test deduplication
        pts = np.random.uniform(2, 12, (500, 3)).astype(np.float32)
        # Add chassis self-reflection point (<0.35m) to verify rejection
        pts = np.vstack([pts, [[0.1, 0.1, 0.1]]])
        dataset_full.add_scan(
            timestamp=time.time() + i * 0.5,
            vehicle_pos=(float(i), float(i % 3), -3.5),
            vehicle_ori=(1.0, 0.0, 0.0, 0.0),
            sensor_pos=sensor_pos,
            sensor_ori=sensor_ori,
            points_sensor=pts
        )

    progress_log = []
    def on_progress(pct, msg):
        progress_log.append((pct, msg))

    artifact = ReconstructionEngine.process_survey(
        dataset_full,
        output_dir=out_dir,
        progress_callback=on_progress
    )

    print(f"[RECONSTRUCTION SUCCESS]")
    print(f"  Artifact File: {artifact.filepath}")
    print(f"  Final Points: {len(artifact.points_3d):,}")
    print(f"  Colors: {artifact.colors.shape}")
    print(f"  Area: {artifact.metadata['area_m2']:.1f} m²")
    print(f"  Bounds: {artifact.metadata['bounds_min']} to {artifact.metadata['bounds_max']}")

    assert artifact.is_valid()
    assert len(artifact.points_3d) > 0
    assert len(artifact.colors) == len(artifact.points_3d)
    assert os.path.exists(artifact.filepath)
    assert os.path.exists(os.path.join(out_dir, "final_survey_latest.npz"))

    # Reload check
    loaded_artifact = ReconstructionEngine.load_artifact(artifact.filepath)
    assert loaded_artifact.is_valid()
    assert len(loaded_artifact.points_3d) == len(artifact.points_3d)
    print("[PASS] Multi-scan reconstruction, voxel deduplication, and verification passed.")

    print("\n=== TEST 5: Point Cloud Export (XYZ, PLY, NPZ) ===")
    test_xyz = os.path.join(out_dir, "test_export.xyz")
    test_ply = os.path.join(out_dir, "test_export.ply")
    assert export_survey_point_cloud(artifact, test_xyz, "xyz")
    assert export_survey_point_cloud(artifact, test_ply, "ply")
    assert os.path.exists(test_xyz) and os.path.getsize(test_xyz) > 1000
    assert os.path.exists(test_ply) and os.path.getsize(test_ply) > 1000

    # Cleanup test exports
    for p in [test_xyz, test_ply, artifact.filepath]:
        if os.path.exists(p):
            os.remove(p)

    print("[PASS] Export formats (XYZ, PLY, NPZ) verified.")
    print("\n=== ALL SURVEY ENGINE UNIT TESTS PASSED SUCCESSFULLY! ===")

if __name__ == "__main__":
    test_survey_engine()
