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
    
    from ui.main import LifeMAS, MappingViewportWidget
    
    window = LifeMAS()
    window.show()
    app.processEvents()
    
    print("Window created and shown successfully!")
    print("Testing 3D Mapping Viewport point cloud update...")
    import numpy as np
    sample_points = np.random.uniform(-10, 10, (500, 3)).astype(np.float32)
    window.mapping_vp.add_point_cloud(sample_points, "Lidar1", 500, 0.0, 0.0, 5.0)
    app.processEvents()
    print("Point cloud render test complete! Accumulated points:", len(window.mapping_vp.accumulated_points))
    
    window.mapping_vp.reset_map()
    app.processEvents()
    print("Reset map test complete! Points after reset:", len(window.mapping_vp.accumulated_points))
    
    window.close()
    app.processEvents()
    print("UI Launch & Component Verification PASSED cleanly.")

if __name__ == "__main__":
    test_launch()
