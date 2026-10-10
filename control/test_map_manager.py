import sys
import os
import numpy as np

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from mapping.persistent_map import PersistentMapManager

def test_map_manager():
    print("=== Testing PersistentMapManager ===")
    mapper = PersistentMapManager(cell_size=0.15, voxel_size=0.08)
    
    # 1. Add scan at Origin (0, 0, 0)
    pts1 = np.array([
        [5.0, 0.0, 1.0],
        [0.0, 5.0, 1.0],
        [-5.0, 0.0, 1.0]
    ], dtype=np.float32)
    
    mapper.add_lidar_scan(pts1, drone_x=0.0, drone_y=0.0, drone_z=0.0)
    
    grid1, min_gx1, max_gx1, min_gy1, max_gy1, cell_sz = mapper.get_2d_occupancy_grid()
    pts_3d_1 = mapper.get_3d_points()
    
    print(f"Grid 1 shape: {grid1.shape}, 3D points count: {len(pts_3d_1)}")
    assert len(pts_3d_1) == 3
    assert np.any(grid1 == 2) # Occupied cells present
    assert np.any(grid1 == 1) # Free cells present

    # 2. Add scan at Drone Position (10, 10, 3)
    pts2 = np.array([
        [15.0, 10.0, 2.0],
        [10.0, 15.0, 2.0],
        [5.0, 0.0, 1.0] # Duplicate point from scan 1 to test voxel deduplication
    ], dtype=np.float32)

    mapper.add_lidar_scan(pts2, drone_x=10.0, drone_y=10.0, drone_z=3.0)
    
    grid2, min_gx2, max_gx2, min_gy2, max_gy2, _ = mapper.get_2d_occupancy_grid()
    pts_3d_2 = mapper.get_3d_points()

    print(f"Grid 2 shape: {grid2.shape}, 3D points count: {len(pts_3d_2)}")
    # Scan 1 had 3 pts, Scan 2 added 2 new pts + 1 duplicate -> total 5 unique voxel points
    assert len(pts_3d_2) == 5
    
    # Verify points from Scan 1 remain at original coordinates [5, 0, 1]
    matching = np.isclose(pts_3d_2, [5.0, 0.0, 1.0]).all(axis=1)
    assert np.any(matching), "Original points must remain at fixed world coordinates!"

    # 3. Test Save & Load
    save_file = os.path.join(project_root, "data", "maps", "test_map.npz")
    mapper.save_map(save_file)
    assert os.path.exists(save_file)
    print(f"[PASS] Saved map to: {save_file}")

    mapper_new = PersistentMapManager()
    loaded = mapper_new.load_map(save_file)
    assert loaded
    assert len(mapper_new.get_3d_points()) == 5
    print("[PASS] Loaded map verified!")

    # Clean up test map file
    if os.path.exists(save_file):
        os.remove(save_file)

    print("\n=== PersistentMapManager Unit Tests PASSED cleanly! ===")

if __name__ == "__main__":
    test_map_manager()
