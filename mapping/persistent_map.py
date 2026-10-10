import os
import math
import numpy as np

class PersistentMapManager:
    """
    Persistent World-Frame Mapping Engine.
    Combines a Robot-Vacuum style 2D Occupancy Grid with a 3D Voxel Point Cloud.
    All data is stored in a fixed world coordinate frame (AirSim World Coordinates).
    """
    def __init__(self, cell_size=0.15, voxel_size=0.08):
        self.cell_size = float(cell_size)      # 15 cm 2D grid resolution
        self.voxel_size = float(voxel_size)    # 8 cm 3D voxel resolution
        
        # 2D Occupancy Grid: dict mapping (grid_x, grid_y) -> cell_state (1: FREE, 2: OCCUPIED)
        self.grid_cells = {}
        
        # 3D Voxel Point Cloud: dict mapping (vx, vy, vz) -> np.array([x, y, z], dtype=np.float32)
        self.voxel_points = {}
        
        # World Bounds tracking
        self.min_x = 0.0
        self.max_x = 0.0
        self.min_y = 0.0
        self.max_y = 0.0
        self.has_bounds = False

    def world_to_grid(self, x, y):
        gx = int(math.floor(x / self.cell_size))
        gy = int(math.floor(y / self.cell_size))
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        x = (gx + 0.5) * self.cell_size
        y = (gy + 0.5) * self.cell_size
        return (x, y)

    def world_to_voxel(self, x, y, z):
        vx = int(math.floor(x / self.voxel_size))
        vy = int(math.floor(y / self.voxel_size))
        vz = int(math.floor(z / self.voxel_size))
        return (vx, vy, vz)

    def bresenham_2d(self, x0, y0, x1, y1):
        """2D Bresenham ray-tracing from (x0, y0) to (x1, y1)."""
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)

        cells = []
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy

        curr_x, curr_y = gx0, gy0
        while True:
            cells.append((curr_x, curr_y))
            if curr_x == gx1 and curr_y == gy1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                curr_x += sx
            if e2 < dx:
                err += dx
                curr_y += sy
        return cells

    def add_lidar_scan(self, pts_vis, drone_x, drone_y, drone_z):
        """
        Integrate a batch of world-transformed 3D LiDAR points.
        pts_vis is (N, 3) in Visualization World Coordinates (X North, Y East, Z Up).
        """
        if len(pts_vis) == 0:
            return

        # Update world bounds
        xs = pts_vis[:, 0]
        ys = pts_vis[:, 1]
        b_min_x, b_max_x = float(np.min(xs)), float(np.max(xs))
        b_min_y, b_max_y = float(np.min(ys)), float(np.max(ys))

        if not self.has_bounds:
            self.min_x, self.max_x = min(b_min_x, drone_x), max(b_max_x, drone_x)
            self.min_y, self.max_y = min(b_min_y, drone_y), max(b_max_y, drone_y)
            self.has_bounds = True
        else:
            self.min_x = min(self.min_x, b_min_x, drone_x)
            self.max_x = max(self.max_x, b_max_x, drone_x)
            self.min_y = min(self.min_y, b_min_y, drone_y)
            self.max_y = max(self.max_y, b_max_y, drone_y)

        # 1. Integrate 3D Voxel Points (Deduplicated spatial storage)
        for pt in pts_vis:
            v_key = self.world_to_voxel(pt[0], pt[1], pt[2])
            if v_key not in self.voxel_points:
                self.voxel_points[v_key] = pt.astype(np.float32)

        # 2. Integrate 2D Occupancy Grid (Subsample points for efficient ray tracing)
        step = max(1, len(pts_vis) // 150)
        sample_pts = pts_vis[::step]

        for pt in sample_pts:
            px, py, pz = pt[0], pt[1], pt[2]
            dist_2d = math.hypot(px - drone_x, py - drone_y)
            
            # Filter zero-distance or noise returns
            if dist_2d < 0.3 or dist_2d > 45.0:
                continue

            hit_gx, hit_gy = self.world_to_grid(px, py)

            # Ray trace free space along line from drone to hit point
            ray_cells = self.bresenham_2d(drone_x, drone_y, px, py)
            for cell in ray_cells[:-1]:
                if self.grid_cells.get(cell) != 2: # Preserve occupied cells
                    self.grid_cells[cell] = 1 # FREE

            # Mark hit cell as OCCUPIED
            self.grid_cells[(hit_gx, hit_gy)] = 2 # OCCUPIED

    def get_3d_points(self):
        if not self.voxel_points:
            return np.empty((0, 3), dtype=np.float32)
        return np.array(list(self.voxel_points.values()), dtype=np.float32)

    def get_2d_occupancy_grid(self):
        """Returns (grid_matrix, min_gx, max_gx, min_gy, max_gy, cell_size)."""
        if not self.grid_cells:
            return np.zeros((1, 1), dtype=np.uint8), 0, 0, 0, 0, self.cell_size

        gxs = [k[0] for k in self.grid_cells.keys()]
        gys = [k[1] for k in self.grid_cells.keys()]
        min_gx, max_gx = min(gxs) - 5, max(gxs) + 5
        min_gy, max_gy = min(gys) - 5, max(gys) + 5

        width = max_gx - min_gx + 1
        height = max_gy - min_gy + 1

        grid = np.zeros((height, width), dtype=np.uint8)
        for (gx, gy), state in self.grid_cells.items():
            row = gy - min_gy
            col = gx - min_gx
            if 0 <= row < height and 0 <= col < width:
                grid[row, col] = state

        return grid, min_gx, max_gx, min_gy, max_gy, self.cell_size

    def clear(self):
        self.grid_cells.clear()
        self.voxel_points.clear()
        self.has_bounds = False
        self.min_x = self.max_x = self.min_y = self.max_y = 0.0

    def save_map(self, filepath):
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        tmp_path = filepath.replace(".npz", "_tmp.npz") if filepath.endswith(".npz") else filepath + "_tmp.npz"
        
        if self.grid_cells:
            keys = np.array(list(self.grid_cells.keys()), dtype=np.int32)
            vals = np.array(list(self.grid_cells.values()), dtype=np.uint8)
        else:
            keys = np.empty((0, 2), dtype=np.int32)
            vals = np.empty((0,), dtype=np.uint8)

        pts_3d = self.get_3d_points()
        metadata = np.array([self.cell_size, self.voxel_size], dtype=np.float64)

        np.savez_compressed(
            tmp_path,
            grid_keys=keys,
            grid_vals=vals,
            points_3d=pts_3d,
            meta=metadata
        )
        os.replace(tmp_path, filepath)

    def load_map(self, filepath):
        if not os.path.exists(filepath):
            return False
        
        data = np.load(filepath)
        keys = data['grid_keys']
        vals = data['grid_vals']
        pts_3d = data['points_3d']
        meta = data['meta']

        self.cell_size = float(meta[0])
        self.voxel_size = float(meta[1])

        # Merge into existing cells without clearing or shifting origin
        for k, v in zip(keys, vals):
            self.grid_cells[(int(k[0]), int(k[1]))] = int(v)

        for pt in pts_3d:
            v_key = self.world_to_voxel(pt[0], pt[1], pt[2])
            if v_key not in self.voxel_points:
                self.voxel_points[v_key] = pt.astype(np.float32)

        if len(pts_3d) > 0:
            xs = pts_3d[:, 0]
            ys = pts_3d[:, 1]
            if not self.has_bounds:
                self.min_x, self.max_x = float(np.min(xs)), float(np.max(xs))
                self.min_y, self.max_y = float(np.min(ys)), float(np.max(ys))
                self.has_bounds = True
            else:
                self.min_x = min(self.min_x, float(np.min(xs)))
                self.max_x = max(self.max_x, float(np.max(xs)))
                self.min_y = min(self.min_y, float(np.min(ys)))
                self.max_y = max(self.max_y, float(np.max(ys)))

        return True
