import os
import math
import threading
import numpy as np
from PySide6.QtGui import QImage

class PersistentMapManager:
    """
    High-Performance Thread-Safe Persistent World-Frame Mapping Engine.
    - 2D Occupancy Grid (Robot-Vacuum style with ground/obstacle discrimination).
    - 3D Voxel-Deduplicated Point Cloud.
    - Vectorized hardware-accelerated QImage rasterization (sub-millisecond).
    - Atomic disk persistence (NPZ format).
    """
    def __init__(self, cell_size=0.15, voxel_size=0.10, ground_threshold=0.35):
        self.cell_size = float(cell_size)              # 15 cm 2D grid resolution
        self.voxel_size = float(voxel_size)            # 10 cm 3D voxel resolution
        self.ground_threshold = float(ground_threshold)# 35 cm height filter above ground
        
        self.lock = threading.RLock()
        
        # 2D Occupancy Grid: (gx, gy) -> state (1: FREE, 2: OCCUPIED)
        self.grid_cells = {}
        
        # 3D Voxel Point Cloud: (vx, vy, vz) -> np.array([x, y, z], dtype=np.float32)
        self.voxel_points = {}
        
        # World bounds tracking
        self.min_x = 0.0
        self.max_x = 0.0
        self.min_y = 0.0
        self.max_y = 0.0
        self.has_bounds = False

        # Ground level tracking in visual coordinates (Z-up)
        self.ground_z = -5.0 # Initial conservative ground level estimate

        # Cached QImage raster layer & dirty flag
        self._cached_qimage = None
        self._cached_bounds = None
        self._is_dirty = True

        # Pre-allocated NumPy color palette for sub-millisecond QImage generation:
        # Index 0: Unknown (#080a0e, dark background)
        # Index 1: Free Space (#1e293b, subtle slate gray)
        # Index 2: Occupied Obstacle (#38bdf8, high-contrast bright cyan)
        self.palette = np.array([
            [8, 10, 14, 255],     # 0: UNKNOWN
            [30, 41, 59, 255],    # 1: FREE
            [56, 189, 248, 255]   # 2: OCCUPIED
        ], dtype=np.uint8)

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

    def bresenham_2d(self, gx0, gy0, gx1, gy1):
        """Fast integer 2D Bresenham ray-tracing."""
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
        Thread-safe integration of a batch of world-transformed 3D LiDAR points.
        pts_vis is (N, 3) in Visualization World Coordinates (X North, Y East, Z Up).
        Runs entirely in background worker thread!
        """
        if len(pts_vis) == 0:
            return

        with self.lock:
            # Update ground level estimate
            z_vals = pts_vis[:, 2]
            if len(pts_vis) >= 20:
                lowest_z = float(np.percentile(z_vals, 5))
                if not self.has_bounds or lowest_z < self.ground_z:
                    self.ground_z = lowest_z
            else:
                lowest_z = float(np.min(z_vals))
                if not self.has_bounds:
                    self.ground_z = min(drone_z - 0.5, lowest_z - 0.5)

            obstacle_cutoff = self.ground_z + self.ground_threshold

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

            # 1. 3D Voxel Points Integration (Spatial deduplication)
            for pt in pts_vis:
                v_key = self.world_to_voxel(pt[0], pt[1], pt[2])
                if v_key not in self.voxel_points:
                    self.voxel_points[v_key] = pt.astype(np.float32)

            # 2. 2D Occupancy Grid Integration with Ground vs Obstacle Discrimination
            drone_gx, drone_gy = self.world_to_grid(drone_x, drone_y)

            # Subsample for efficient ray tracing
            step = max(1, len(pts_vis) // 180)
            sample_pts = pts_vis[::step]

            for pt in sample_pts:
                px, py, pz = pt[0], pt[1], pt[2]
                dist_2d = math.hypot(px - drone_x, py - drone_y)
                
                # Filter invalid zero returns or distant noise
                if dist_2d < 0.25 or dist_2d > 45.0:
                    continue

                hit_gx, hit_gy = self.world_to_grid(px, py)
                ray_cells = self.bresenham_2d(drone_gx, drone_gy, hit_gx, hit_gy)

                # Ray path is confirmed FREE
                for cell in ray_cells[:-1]:
                    if self.grid_cells.get(cell) != 2:
                        self.grid_cells[cell] = 1 # FREE

                # Endpoint classification:
                # If hit is higher than ground threshold -> OCCUPIED OBSTACLE
                # If hit is at or below ground threshold -> Traversable GROUND (marked FREE)
                if pz >= obstacle_cutoff and pz <= (drone_z + 3.0):
                    self.grid_cells[(hit_gx, hit_gy)] = 2 # OCCUPIED
                else:
                    if self.grid_cells.get((hit_gx, hit_gy)) != 2:
                        self.grid_cells[(hit_gx, hit_gy)] = 1 # FREE GROUND

            self._is_dirty = True

    def get_3d_points(self):
        with self.lock:
            if not self.voxel_points:
                return np.empty((0, 3), dtype=np.float32)
            return np.array(list(self.voxel_points.values()), dtype=np.float32)

    def get_occupancy_qimage(self):
        """
        Generates a cached QImage representation of the 2D occupancy grid in <1ms.
        Returns: (qimage, world_min_x, world_max_x, world_min_y, world_max_y, cell_size)
        """
        with self.lock:
            if not self.grid_cells:
                return None, 0.0, 0.0, 0.0, 0.0, self.cell_size

            if not self._is_dirty and self._cached_qimage is not None:
                return self._cached_qimage, *self._cached_bounds, self.cell_size

            # Determine grid extent
            gxs = [k[0] for k in self.grid_cells.keys()]
            gys = [k[1] for k in self.grid_cells.keys()]
            min_gx, max_gx = min(gxs) - 4, max(gxs) + 4
            min_gy, max_gy = min(gys) - 4, max(gys) + 4

            width = max_gx - min_gx + 1
            height = max_gy - min_gy + 1

            # Build fast NumPy 2D state matrix
            grid = np.zeros((height, width), dtype=np.uint8)
            for (gx, gy), state in self.grid_cells.items():
                r = gy - min_gy
                c = gx - min_gx
                if 0 <= r < height and 0 <= c < width:
                    grid[r, c] = state

            # Vectorized Palette Mapping: (H, W, 4) in C speed!
            rgba_matrix = self.palette[grid]
            
            # Flip Y row order so row 0 is top of screen in Qt coordinates
            rgba_flipped = np.ascontiguousarray(np.flipud(rgba_matrix))

            qimage = QImage(
                rgba_flipped.data,
                width,
                height,
                width * 4,
                QImage.Format.Format_RGBA8888
            ).copy()

            world_min_x = min_gx * self.cell_size
            world_max_x = (max_gx + 1) * self.cell_size
            world_min_y = min_gy * self.cell_size
            world_max_y = (max_gy + 1) * self.cell_size

            self._cached_qimage = qimage
            self._cached_bounds = (world_min_x, world_max_x, world_min_y, world_max_y)
            self._is_dirty = False

            return qimage, world_min_x, world_max_x, world_min_y, world_max_y, self.cell_size

    def get_2d_occupancy_grid(self):
        """
        Returns (grid_array, min_gx, max_gx, min_gy, max_gy, cell_size)
        where grid_array is a 2D uint8 numpy array with values:
        0 (unknown), 1 (free), 2 (occupied).
        """
        with self.lock:
            if not self.grid_cells:
                return np.zeros((1, 1), dtype=np.uint8), 0, 0, 0, 0, self.cell_size

            gxs = [k[0] for k in self.grid_cells.keys()]
            gys = [k[1] for k in self.grid_cells.keys()]
            min_gx, max_gx = min(gxs), max(gxs)
            min_gy, max_gy = min(gys), max(gys)

            width = max_gx - min_gx + 1
            height = max_gy - min_gy + 1

            grid = np.zeros((height, width), dtype=np.uint8)
            for (gx, gy), state in self.grid_cells.items():
                r = gy - min_gy
                c = gx - min_gx
                if 0 <= r < height and 0 <= c < width:
                    grid[r, c] = state

            return grid, min_gx, max_gx, min_gy, max_gy, self.cell_size

    def clear(self):
        with self.lock:
            self.grid_cells.clear()
            self.voxel_points.clear()
            self.has_bounds = False
            self.min_x = self.max_x = self.min_y = self.max_y = 0.0
            self.ground_z = -5.0
            self._cached_qimage = None
            self._cached_bounds = None
            self._is_dirty = True

    def save_map(self, filepath):
        """Atomic safe disk persistence using temporary file and atomic replace."""
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        tmp_path = filepath.replace(".npz", "_tmp.npz") if filepath.endswith(".npz") else filepath + "_tmp.npz"

        with self.lock:
            if self.grid_cells:
                keys = np.array(list(self.grid_cells.keys()), dtype=np.int32)
                vals = np.array(list(self.grid_cells.values()), dtype=np.uint8)
            else:
                keys = np.empty((0, 2), dtype=np.int32)
                vals = np.empty((0,), dtype=np.uint8)

            pts_3d = self.get_3d_points()
            metadata = np.array([self.cell_size, self.voxel_size, self.ground_threshold], dtype=np.float64)

            np.savez_compressed(
                tmp_path,
                grid_keys=keys,
                grid_vals=vals,
                points_3d=pts_3d,
                meta=metadata
            )
            os.replace(tmp_path, filepath)

    def load_map(self, filepath):
        """Loads and merges saved map from disk without clearing existing unconflicting observations."""
        if not os.path.exists(filepath):
            return False

        with self.lock:
            try:
                data = np.load(filepath)
                keys = data['grid_keys']
                vals = data['grid_vals']
                pts_3d = data['points_3d']
                meta = data['meta']

                self.cell_size = float(meta[0])
                self.voxel_size = float(meta[1])
                if len(meta) > 2:
                    self.ground_threshold = float(meta[2])

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

                self._is_dirty = True
                return True
            except Exception as e:
                print(f"[ERROR] Failed to load map from {filepath}: {e}")
                return False
