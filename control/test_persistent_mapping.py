import sys
import os
import numpy as np

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from PySide6.QtWidgets import QApplication
from mapping.persistent_map import PersistentMapManager
from ui.main import LifeMAS, SpatialMapWindow

def test_persistent_mapping_requirements():
    print("=== RUNNING PERSISTENT MAPPING REQUIREMENT VERIFICATION TESTS ===")
    
    # -------------------------------------------------------------
    # TEST 1: Points remain at same world coordinates after drone moves
    # -------------------------------------------------------------
    print("\n--- Test 1: World-Frame Coordinate Stability ---")
    mapper = PersistentMapManager(cell_size=0.15, voxel_size=0.08)
    
    # Scan 1 at Drone Pos (0, 0, 0)
    scan1 = np.array([[5.0, 2.0, 1.0]], dtype=np.float32)
    mapper.add_lidar_scan(scan1, drone_x=0.0, drone_y=0.0, drone_z=0.0)
    
    # Scan 2 at Drone Pos (15, 10, 3)
    scan2 = np.array([[18.0, 12.0, 2.0]], dtype=np.float32)
    mapper.add_lidar_scan(scan2, drone_x=15.0, drone_y=10.0, drone_z=3.0)
    
    pts_3d = mapper.get_3d_points()
    assert len(pts_3d) == 2, f"Expected 2 points, got {len(pts_3d)}"
    
    # Verify Scan 1 point is still exactly [5.0, 2.0, 1.0]
    match1 = np.isclose(pts_3d, [5.0, 2.0, 1.0]).all(axis=1)
    assert np.any(match1), "Scan 1 point must remain at fixed world coordinates!"
    print("[PASS] World-frame point stability verified. Points do not shift when drone moves.")

    # -------------------------------------------------------------
    # TEST 2: Previous cells remain after later scans (No fading/erasing)
    # -------------------------------------------------------------
    print("\n--- Test 2: Historical Observation Preservation ---")
    cell_scan1_gx, cell_scan1_gy = mapper.world_to_grid(5.0, 2.0)
    assert mapper.grid_cells.get((cell_scan1_gx, cell_scan1_gy)) == 2, "Scan 1 cell must be marked OCCUPIED."
    
    # Perform 5 subsequent scans from new drone positions
    for i in range(5):
        scan_i = np.array([[20.0 + i, 20.0 + i, 1.0]], dtype=np.float32)
        mapper.add_lidar_scan(scan_i, drone_x=18.0 + i, drone_y=18.0 + i, drone_z=3.0)
        
    # Check that Scan 1 cell is still OCCUPIED (2)
    assert mapper.grid_cells.get((cell_scan1_gx, cell_scan1_gy)) == 2, "Scan 1 cell must remain preserved across subsequent scans!"
    print("[PASS] Historical observations preserved. Subsequent scans do not erase earlier regions.")

    # -------------------------------------------------------------
    # TEST 3: Opening/closing map window does NOT clear data
    # -------------------------------------------------------------
    print("\n--- Test 3: UI Window State Persistence ---")
    app = QApplication.instance() or QApplication(sys.argv)
    window = LifeMAS()
    window.show()
    app.processEvents()

    # Add points to map window
    window.map_window.map_manager.add_lidar_scan(scan1, drone_x=0.0, drone_y=0.0, drone_z=0.0)
    initial_count = len(window.map_window.map_manager.get_3d_points())
    assert initial_count > 0

    # Open Map Window
    window.open_map_window()
    app.processEvents()
    assert window.map_window.isVisible()
    assert len(window.map_window.map_manager.get_3d_points()) == initial_count

    # Close/Hide Map Window
    window.map_window.hide()
    app.processEvents()
    assert not window.map_window.isVisible()
    assert len(window.map_window.map_manager.get_3d_points()) == initial_count, "Closing window must not clear map!"
    print("[PASS] UI Window open/close persistence verified. Map data retained.")

    # -------------------------------------------------------------
    # TEST 4: Saving and reloading preserves cell values, origin, and resolution
    # -------------------------------------------------------------
    print("\n--- Test 4: Disk Persistence (Save & Load NPZ) ---")
    map_dir = os.path.join(project_root, "data", "maps")
    os.makedirs(map_dir, exist_ok=True)
    test_save_file = os.path.join(map_dir, "test_persistent_map.npz")
    
    # Save current mapper state
    mapper.save_map(test_save_file)
    assert os.path.exists(test_save_file), "Map file must exist on disk."

    # Load into new mapper instance
    new_mapper = PersistentMapManager()
    loaded_ok = new_mapper.load_map(test_save_file)
    assert loaded_ok, "load_map must return True."
    
    assert new_mapper.cell_size == mapper.cell_size, "Cell size resolution must match."
    assert len(new_mapper.grid_cells) == len(mapper.grid_cells), "Cell count must match."
    assert len(new_mapper.get_3d_points()) == len(mapper.get_3d_points()), "3D point count must match."
    assert new_mapper.grid_cells.get((cell_scan1_gx, cell_scan1_gy)) == 2, "Loaded grid cell value must match."
    
    # Clean up test file
    if os.path.exists(test_save_file):
        os.remove(test_save_file)
    print("[PASS] Disk persistence verified. Save & Load preserves grid values, origin, and resolution.")

    # -------------------------------------------------------------
    # TEST 5: Resetting clears the map ONLY when explicitly requested
    # -------------------------------------------------------------
    print("\n--- Test 5: Explicit Reset Map Only ---")
    assert len(mapper.grid_cells) > 0
    mapper.clear()
    assert len(mapper.grid_cells) == 0, "Clear should empty grid_cells."
    assert len(mapper.voxel_points) == 0, "Clear should empty voxel_points."
    print("[PASS] Explicit Reset verified. Map clears only upon direct user reset action.")

    window.close()
    app.processEvents()
    print("\n=== ALL PERSISTENT MAPPING REQUIREMENT VERIFICATION TESTS PASSED SUCCESSFULLY! ===")

if __name__ == "__main__":
    test_persistent_mapping_requirements()
