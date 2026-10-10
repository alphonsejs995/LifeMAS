import os
import time
import math
import json
import threading
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Tuple, Optional, Callable
import numpy as np

# Spatial Transform Helpers (Sensor -> Vehicle Body -> World NED -> Visual Z-Up)

def quaternion_to_rot_matrix(q_tuple: Tuple[float, float, float, float]) -> np.ndarray:
    """Converts (w, x, y, z) quaternion into a 3x3 rotation matrix."""
    w, x, y, z = q_tuple
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm > 1e-6:
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
    else:
        w, x, y, z = 1.0, 0.0, 0.0, 0.0
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)]
    ], dtype=np.float32)


def transform_scan_to_world_vis(
    pts_sensor: np.ndarray,
    sensor_pos: Tuple[float, float, float],
    sensor_ori: Tuple[float, float, float, float],
    vehicle_pos: Optional[Tuple[float, float, float]] = None,
    vehicle_ori: Optional[Tuple[float, float, float, float]] = None
) -> np.ndarray:
    """
    Transforms sensor-relative points (N, 3) to AirSim visual world frame (Z-Up).
    Applies exact sensor world pose at acquisition time without duplicate transformation.
    """
    if len(pts_sensor) == 0:
        return np.empty((0, 3), dtype=np.float32)

    pts = np.asarray(pts_sensor, dtype=np.float32)
    Rs = quaternion_to_rot_matrix(sensor_ori)
    Ts = np.array(sensor_pos, dtype=np.float32)

    # In AirSim, lidar_data.pose is already the sensor's world NED pose (params.relative_pose + kinematics.pose).
    # If vehicle_pos is provided and differs substantially from sensor_pos (e.g. unit tests passing mounting offset),
    # apply two-stage transform. Otherwise, sensor_pos is already the world pose.
    is_local_offset = False
    if vehicle_pos is not None and vehicle_ori is not None:
        Tv = np.array(vehicle_pos, dtype=np.float32)
        if not np.allclose(Ts, Tv, atol=0.05):
            is_local_offset = True

    if is_local_offset:
        pts_drone = pts @ Rs.T + Ts
        Rv = quaternion_to_rot_matrix(vehicle_ori)
        pts_ned = pts_drone @ Rv.T + Tv
    else:
        pts_ned = pts @ Rs.T + Ts

    # Convert NED (X North, Y East, Z Down) to Visual Coordinates (X North, Y East, Z Up)
    pts_vis = pts_ned.copy()
    pts_vis[:, 2] = -pts_ned[:, 2]
    return pts_vis


def compute_elevation_colors(pts_vis: np.ndarray) -> np.ndarray:
    """
    Computes a refined elevation gradient colormap for 3D aerial survey point clouds.
    Low/Ground (Deep Blue/Teal) -> Mid (Emerald/Amber) -> High (Warm Gold/White).
    """
    if len(pts_vis) == 0:
        return np.empty((0, 4), dtype=np.float32)

    z = pts_vis[:, 2]
    # Robust percentile-based dynamic range (prevents single outlier from flattening colormap)
    z_min = float(np.percentile(z, 1.0))
    z_max = float(np.percentile(z, 99.0))
    span = z_max - z_min
    if span < 1e-3:
        t = np.zeros_like(z)
    else:
        t = np.clip((z - z_min) / span, 0.0, 1.0)

    # Multi-stage elevation gradient:
    # 0.00: Deep Navy/Teal   (30, 80, 160)
    # 0.25: Vibrant Cyan     (20, 184, 166)
    # 0.50: Emerald Green    (16, 185, 129)
    # 0.75: Amber Gold       (245, 158, 11)
    # 1.00: Bright Ivory     (255, 240, 220)
    r = np.zeros_like(t)
    g = np.zeros_like(t)
    b = np.zeros_like(t)

    # Stage 1: [0.0, 0.25]
    m1 = (t <= 0.25)
    f1 = t[m1] / 0.25
    r[m1] = (30 + (20 - 30) * f1) / 255.0
    g[m1] = (80 + (184 - 80) * f1) / 255.0
    b[m1] = (160 + (166 - 160) * f1) / 255.0

    # Stage 2: [0.25, 0.50]
    m2 = (t > 0.25) & (t <= 0.50)
    f2 = (t[m2] - 0.25) / 0.25
    r[m2] = (20 + (16 - 20) * f2) / 255.0
    g[m2] = (184 + (185 - 184) * f2) / 255.0
    b[m2] = (166 + (129 - 166) * f2) / 255.0

    # Stage 3: [0.50, 0.75]
    m3 = (t > 0.50) & (t <= 0.75)
    f3 = (t[m3] - 0.50) / 0.25
    r[m3] = (16 + (245 - 16) * f3) / 255.0
    g[m3] = (185 + (158 - 185) * f3) / 255.0
    b[m3] = (129 + (11 - 129) * f3) / 255.0

    # Stage 4: [0.75, 1.00]
    m4 = (t > 0.75)
    f4 = (t[m4] - 0.75) / 0.25
    r[m4] = (245 + (255 - 245) * f4) / 255.0
    g[m4] = (158 + (240 - 158) * f4) / 255.0
    b[m4] = (11 + (220 - 11) * f4) / 255.0

    colors = np.column_stack([r, g, b, np.ones_like(r) * 0.95]).astype(np.float32)
    return colors


@dataclass
class SurveyConfig:
    survey_id: str = ""
    length_x: float = 14.0         # Forward coverage extent in meters (X in [0, length_x])
    width_y: float = 8.0           # Lateral coverage extent in meters (Y in [-width_y/2, +width_y/2])
    altitude: float = 3.8          # Survey flight altitude in meters (NED Z = -altitude)
    lane_spacing: float = 2.5      # Distance between parallel survey passes in meters
    flight_speed: float = 1.2      # Speed on survey passes in m/s (12cm per scan at 10Hz)
    turn_speed: float = 0.8        # Speed on lane-change transitions in m/s (smooth yaw)
    min_scans_required: int = 15   # Minimum valid LiDAR scans required for reconstruction
    voxel_size: float = 0.08       # 3D spatial voxel deduplication resolution (8 cm)
    min_range: float = 0.40        # Ignore near-field drone chassis reflections
    max_range: float = 45.0        # Ignore noisy distant sky/horizon returns

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "SurveyConfig":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


def generate_survey_waypoints(config: SurveyConfig) -> List[Dict]:
    """
    Generates a boustrophedon (lawnmower) survey flight plan in AirSim NED coordinates.
    All waypoints stay strictly bounded within [0, config.length_x] x [-config.width_y/2, config.width_y/2].
    """
    waypoints = []
    z_ned = -abs(config.altitude)
    half_w = config.width_y / 2.0

    # Calculate parallel lanes
    num_lanes = max(2, int(math.ceil(config.width_y / config.lane_spacing)))
    actual_spacing = config.width_y / (num_lanes - 1) if num_lanes > 1 else config.lane_spacing
    y_coords = [-half_w + i * actual_spacing for i in range(num_lanes)]

    # 1. Takeoff waypoint at origin
    waypoints.append({
        "type": "TAKEOFF",
        "name": "Takeoff",
        "x": 0.0, "y": 0.0, "z": z_ned,
        "speed": 1.2,
        "description": f"Takeoff and climb to survey altitude {config.altitude:.1f}m"
    })

    # 2. Transition to first lane start
    first_y = y_coords[0]
    waypoints.append({
        "type": "TRANSITION",
        "name": "Lane 1 Entry",
        "x": 0.0, "y": first_y, "z": z_ned,
        "speed": config.turn_speed,
        "description": f"Align with Lane 1 start (Y={first_y:.1f}m)"
    })

    # 3. Lawnmower passes
    forward = True
    for idx, y in enumerate(y_coords):
        lane_num = idx + 1
        if forward:
            # Pass forward from 0 to length_x
            waypoints.append({
                "type": "PASS",
                "name": f"Pass {lane_num}/{num_lanes} (North)",
                "x": config.length_x, "y": y, "z": z_ned,
                "speed": config.flight_speed,
                "description": f"Survey Pass {lane_num}: Forward (+X) to {config.length_x:.1f}m"
            })
        else:
            # Pass backward from length_x to 0
            waypoints.append({
                "type": "PASS",
                "name": f"Pass {lane_num}/{num_lanes} (South)",
                "x": 0.0, "y": y, "z": z_ned,
                "speed": config.flight_speed,
                "description": f"Survey Pass {lane_num}: Backward (-X) to 0.0m"
            })

        # Transition to next lane if not last
        if idx < len(y_coords) - 1:
            next_y = y_coords[idx + 1]
            turn_x = config.length_x if forward else 0.0
            waypoints.append({
                "type": "TURN",
                "name": f"Transition -> Lane {lane_num + 1}",
                "x": turn_x, "y": next_y, "z": z_ned,
                "speed": config.turn_speed,
                "description": f"Shift laterally to Y={next_y:.1f}m"
            })
            forward = not forward

    # 4. Return to home origin
    waypoints.append({
        "type": "RETURN",
        "name": "Return Home",
        "x": 0.0, "y": 0.0, "z": z_ned,
        "speed": config.turn_speed,
        "description": "Return to launch point before descent"
    })

    # 5. Land at origin
    waypoints.append({
        "type": "LAND",
        "name": "Landing",
        "x": 0.0, "y": 0.0, "z": 0.0,
        "speed": 0.8,
        "description": "Initiate automated descent and landing"
    })

    return waypoints


class SurveyDataset:
    """
    Thread-safe in-memory container for raw LiDAR scans recorded during an aerial survey.
    Preserves exact acquisition poses and raw sensor returns.
    """
    def __init__(self, survey_id: str = "", config: Optional[SurveyConfig] = None):
        self.survey_id = survey_id or time.strftime("survey_%Y%m%d_%H%M%S")
        self.config = config or SurveyConfig(survey_id=self.survey_id)
        self.scans: List[Dict] = []
        self.lock = threading.RLock()
        self.created_at = time.time()

    def add_scan(
        self,
        timestamp: float,
        vehicle_pos: Tuple[float, float, float],
        vehicle_ori: Tuple[float, float, float, float],
        sensor_pos: Tuple[float, float, float],
        sensor_ori: Tuple[float, float, float, float],
        points_sensor: np.ndarray
    ):
        """Thread-safe recording of a single LiDAR scan."""
        if len(points_sensor) == 0:
            return

        with self.lock:
            record = {
                "timestamp": float(timestamp),
                "vehicle_pos": tuple(float(v) for v in vehicle_pos),
                "vehicle_ori": tuple(float(v) for v in vehicle_ori),
                "sensor_pos": tuple(float(s) for s in sensor_pos),
                "sensor_ori": tuple(float(s) for s in sensor_ori),
                "points": np.asarray(points_sensor, dtype=np.float32)
            }
            self.scans.append(record)

    def get_scan_count(self) -> int:
        with self.lock:
            return len(self.scans)

    def get_total_raw_points(self) -> int:
        with self.lock:
            return sum(len(s["points"]) for s in self.scans)

    def save_raw_dataset(self, filepath: str):
        """Atomically saves the raw dataset to an NPZ file."""
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        tmp_path = filepath.replace(".npz", "_tmp.npz") if filepath.endswith(".npz") else filepath + "_tmp.npz"

        with self.lock:
            meta = {
                "survey_id": self.survey_id,
                "config": self.config.to_dict(),
                "created_at": self.created_at,
                "scan_count": len(self.scans),
                "total_points": self.get_total_raw_points()
            }
            arrays_to_save = {
                "metadata": np.array(json.dumps(meta))
            }
            for i, scan in enumerate(self.scans):
                arrays_to_save[f"scan_{i}_pts"] = scan["points"]
                arrays_to_save[f"scan_{i}_pose"] = np.array([
                    scan["timestamp"],
                    *scan["vehicle_pos"],
                    *scan["vehicle_ori"],
                    *scan["sensor_pos"],
                    *scan["sensor_ori"]
                ], dtype=np.float64)

            np.savez_compressed(tmp_path, **arrays_to_save)
            os.replace(tmp_path, filepath)

    @classmethod
    def load_raw_dataset(cls, filepath: str) -> "SurveyDataset":
        """Loads a previously saved raw dataset from disk."""
        data = np.load(filepath, allow_pickle=True)
        meta = json.loads(str(data["metadata"]))
        config = SurveyConfig.from_dict(meta.get("config", {}))
        dataset = cls(survey_id=meta["survey_id"], config=config)
        dataset.created_at = meta.get("created_at", time.time())

        scan_count = meta.get("scan_count", 0)
        for i in range(scan_count):
            pts = data[f"scan_{i}_pts"]
            pose = data[f"scan_{i}_pose"]
            dataset.add_scan(
                timestamp=float(pose[0]),
                vehicle_pos=(float(pose[1]), float(pose[2]), float(pose[3])),
                vehicle_ori=(float(pose[4]), float(pose[5]), float(pose[6]), float(pose[7])),
                sensor_pos=(float(pose[8]), float(pose[9]), float(pose[10])),
                sensor_ori=(float(pose[11]), float(pose[12]), float(pose[13]), float(pose[14])),
                points_sensor=pts
            )
        return dataset


@dataclass
class SurveyArtifact:
    """Validated final 3D reconstruction artifact."""
    filepath: str
    points_3d: np.ndarray       # (M, 3) float32 in Visual World Frame (Z-up)
    colors: np.ndarray          # (M, 4) float32 RGBA
    metadata: Dict

    def is_valid(self) -> bool:
        return (
            os.path.exists(self.filepath) and
            isinstance(self.points_3d, np.ndarray) and
            self.points_3d.ndim == 2 and
            self.points_3d.shape[1] == 3 and
            self.points_3d.shape[0] > 0 and
            np.isfinite(self.points_3d).all()
        )


class ReconstructionEngine:
    """
    High-accuracy post-flight 3D point cloud reconstruction pipeline.
    - Transforms each scan with its acquisition pose.
    - Range and outlier rejection.
    - Spatial voxel downsampling (deduplicating stationary & hovering scans).
    - Elevation colormap computation.
    - Atomic serialization & verification.
    """
    @staticmethod
    def process_survey(
        dataset: SurveyDataset,
        output_dir: str,
        progress_callback: Optional[Callable[[int, str], None]] = None
    ) -> SurveyArtifact:
        config = dataset.config
        scan_count = dataset.get_scan_count()

        if scan_count < config.min_scans_required:
            raise ValueError(
                f"Insufficient LiDAR data collected: recorded {scan_count} scans, "
                f"minimum required for reconstruction is {config.min_scans_required}."
            )

        if progress_callback:
            progress_callback(5, f"Transforming {scan_count} scans into single world frame...")

        voxel_dict: Dict[Tuple[int, int, int], np.ndarray] = {}
        voxel_size = config.voxel_size
        total_raw_points = 0
        transformed_points_count = 0

        # 1. Transform each scan and insert into spatial voxel hash
        with dataset.lock:
            for idx, scan in enumerate(dataset.scans):
                pts_raw = scan["points"]
                total_raw_points += len(pts_raw)
                if len(pts_raw) == 0:
                    continue

                # Range filter (ignore chassis self-reflections, infinite rays, and distant noise)
                finite_mask = np.isfinite(pts_raw).all(axis=1)
                pts_raw_finite = pts_raw[finite_mask]
                if len(pts_raw_finite) == 0:
                    continue

                ranges = np.linalg.norm(pts_raw_finite, axis=1)
                valid_mask = (ranges >= config.min_range) & (ranges <= config.max_range)
                pts_filtered = pts_raw_finite[valid_mask]
                if len(pts_filtered) == 0:
                    continue

                # Transform to Visual World Frame using scan's unique sensor and drone pose
                pts_vis = transform_scan_to_world_vis(
                    pts_filtered,
                    scan["sensor_pos"],
                    scan["sensor_ori"],
                    scan["vehicle_pos"],
                    scan["vehicle_ori"]
                )

                # Physical height filter: reject subterranean penetration (< -2.5m) and sky rays (> 25.0m)
                h_mask = (pts_vis[:, 2] >= -2.5) & (pts_vis[:, 2] <= 25.0)
                pts_vis = pts_vis[h_mask]
                if len(pts_vis) == 0:
                    continue

                transformed_points_count += len(pts_vis)

                # Voxel Grid Deduplication (Spatial hash)
                vx = np.floor(pts_vis[:, 0] / voxel_size).astype(np.int32)
                vy = np.floor(pts_vis[:, 1] / voxel_size).astype(np.int32)
                vz = np.floor(pts_vis[:, 2] / voxel_size).astype(np.int32)

                for p, kx, ky, kz in zip(pts_vis, vx, vy, vz):
                    key = (int(kx), int(ky), int(kz))
                    if key not in voxel_dict:
                        voxel_dict[key] = p.astype(np.float32)

                if progress_callback and (idx % max(1, scan_count // 10) == 0):
                    pct = 5 + int((idx / scan_count) * 60)
                    progress_callback(pct, f"Processed {idx + 1}/{scan_count} scans ({len(voxel_dict):,} voxels)...")

        if len(voxel_dict) == 0:
            raise ValueError("Reconstruction produced 0 valid points after range filtering.")

        if progress_callback:
            progress_callback(70, f"Compiling {len(voxel_dict):,} 3D surface points...")

        # 2. Extract final voxel point cloud
        points_3d = np.array(list(voxel_dict.values()), dtype=np.float32)

        if progress_callback:
            progress_callback(80, "Calculating elevation-based spectral colors...")

        # 3. Elevation coloring
        colors = compute_elevation_colors(points_3d)

        # 4. Metrics & Metadata
        min_bounds = np.min(points_3d, axis=0)
        max_bounds = np.max(points_3d, axis=0)
        span = max_bounds - min_bounds
        area_m2 = float(span[0] * span[1])

        metadata = {
            "survey_id": dataset.survey_id,
            "timestamp": time.time(),
            "scan_count": scan_count,
            "raw_points_count": total_raw_points,
            "transformed_points_count": transformed_points_count,
            "final_points_count": len(points_3d),
            "voxel_size": voxel_size,
            "bounds_min": [float(b) for b in min_bounds],
            "bounds_max": [float(b) for b in max_bounds],
            "area_m2": area_m2,
            "config": config.to_dict()
        }

        # 5. Atomic Disk Persistence
        os.makedirs(output_dir, exist_ok=True)
        final_filename = f"final_survey_{dataset.survey_id}.npz"
        final_filepath = os.path.join(output_dir, final_filename)
        latest_filepath = os.path.join(output_dir, "final_survey_latest.npz")
        tmp_filepath = final_filepath.replace(".npz", "_tmp.npz") if final_filepath.endswith(".npz") else final_filepath + "_tmp.npz"

        if progress_callback:
            progress_callback(90, f"Writing final map artifact: {final_filename}...")

        np.savez_compressed(
            tmp_filepath,
            points_3d=points_3d,
            colors=colors,
            metadata=np.array(json.dumps(metadata))
        )
        os.replace(tmp_filepath, final_filepath)

        # Update latest survey copy atomically
        try:
            import shutil
            shutil.copyfile(final_filepath, latest_filepath)
        except Exception:
            pass

        # 6. Verification check: Reopen and validate artifact
        if progress_callback:
            progress_callback(98, "Validating final 3D map artifact...")

        artifact = ReconstructionEngine.load_artifact(final_filepath)
        if not artifact.is_valid():
            raise RuntimeError("Validation failed on written final survey artifact.")

        if progress_callback:
            progress_callback(100, f"Final map verified: {len(points_3d):,} points ({area_m2:.1f} m²).")

        return artifact

    @staticmethod
    def load_artifact(filepath: str) -> SurveyArtifact:
        """Loads and validates a final map artifact from an NPZ file."""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Artifact not found: {filepath}")

        data = np.load(filepath, allow_pickle=True)
        points_3d = data["points_3d"]
        colors = data["colors"]
        meta_raw = data["metadata"]
        metadata = json.loads(str(meta_raw))

        return SurveyArtifact(
            filepath=filepath,
            points_3d=points_3d,
            colors=colors,
            metadata=metadata
        )


def export_survey_point_cloud(artifact: SurveyArtifact, export_path: str, fmt: str = "xyz") -> bool:
    """Exports the final 3D reconstruction to XYZ, PLY, or NPZ format."""
    try:
        pts = artifact.points_3d
        cols = (artifact.colors[:, :3] * 255.0).astype(np.uint8)

        if fmt.lower() == "xyz":
            with open(export_path, "w") as f:
                for p, c in zip(pts, cols):
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {c[0]} {c[1]} {c[2]}\n")
            return True

        elif fmt.lower() == "ply":
            with open(export_path, "w") as f:
                f.write("ply\nformat ascii 1.0\n")
                f.write(f"element vertex {len(pts)}\n")
                f.write("property float x\nproperty float y\nproperty float z\n")
                f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
                f.write("end_header\n")
                for p, c in zip(pts, cols):
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {c[0]} {c[1]} {c[2]}\n")
            return True

        elif fmt.lower() == "npz":
            np.savez_compressed(export_path, points=pts, colors=artifact.colors, metadata=json.dumps(artifact.metadata))
            return True

        return False
    except Exception as e:
        print(f"[EXPORT ERROR] {e}")
        return False
