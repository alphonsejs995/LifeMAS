import sys
import os

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from PySide6.QtWidgets import QApplication

def test_launch():
    print("Testing LifeMAS UI initialization...")
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    from ui.main import LifeMAS, SpatialMapWindow
    
    window = LifeMAS()
    window.show()
    app.processEvents()
    
    print("Window created and shown successfully!")
    print("Testing SpatialMapWindow open and 2D/3D mapping...")
    window.open_map_window()
    app.processEvents()
    assert window.map_window.isVisible()
    
    import numpy as np
    sample_points = np.random.uniform(-10, 10, (500, 3)).astype(np.float32)
    window.map_window.add_lidar_scan(sample_points, "Lidar1", 500, 0.0, 0.0, 3.0)
    app.processEvents()
    print("Point cloud render test complete! Accumulated points:", len(window.map_window.map_manager.voxel_points))
    
    window.map_window.map_manager.clear()
    app.processEvents()
    print("Reset map test complete! Points after reset:", len(window.map_window.map_manager.voxel_points))
    
    window.map_window.hide()
    window.close()
    app.processEvents()
    print("UI Launch & Component Verification PASSED cleanly.")

if __name__ == "__main__":
    test_launch()
