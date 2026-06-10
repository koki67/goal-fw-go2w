"""Thin wrapper around small_gicp scan-to-map registration.

The map is preprocessed exactly once at load (voxel downsample + covariance
estimation + KdTree); each register() call preprocesses only the live scan.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import small_gicp

from nav_goal_go2w_localization.localization_core import RegistrationOutcome


@dataclass(frozen=True)
class RegistrationConfig:
    registration_type: str = "GICP"  # GICP | VGICP
    map_voxel_size: float = 0.20
    scan_voxel_size: float = 0.25
    num_neighbors: int = 20
    num_threads: int = 4
    max_iterations: int = 20
    vgicp_voxel_resolution: float = 1.0

    def __post_init__(self) -> None:
        if self.registration_type not in ("GICP", "VGICP"):
            raise ValueError("registration_type must be GICP or VGICP")
        if self.map_voxel_size <= 0.0 or self.scan_voxel_size <= 0.0:
            raise ValueError("voxel sizes must be positive")


class MapTarget:
    """Preprocessed map cloud ready for repeated scan registration."""

    def __init__(
        self, map_points_xyz: np.ndarray, config: RegistrationConfig | None = None
    ) -> None:
        self.config = config or RegistrationConfig()
        points = np.asarray(map_points_xyz, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] < 3 or len(points) == 0:
            raise ValueError("map_points_xyz must be a non-empty (N, 3) array")
        self.cloud, self.tree = small_gicp.preprocess_points(
            points[:, :3],
            downsampling_resolution=self.config.map_voxel_size,
            num_neighbors=self.config.num_neighbors,
            num_threads=self.config.num_threads,
        )
        self.voxelmap = None
        if self.config.registration_type == "VGICP":
            self.voxelmap = small_gicp.GaussianVoxelMap(
                self.config.vgicp_voxel_resolution
            )
            self.voxelmap.insert(self.cloud)

    @property
    def size(self) -> int:
        return self.cloud.size()

    def register(
        self,
        source_points_xyz: np.ndarray,
        init_T_map_source: np.ndarray,
        max_correspondence_distance: float,
    ) -> RegistrationOutcome:
        """Register a source cloud against the map.

        ``init_T_map_source`` is the initial guess for the transform from the
        source cloud's frame to the map frame; the returned outcome carries
        the refined transform in ``T_map_odom`` (the source frame is odom for
        the deskewed D-LIO cloud).
        """
        source = np.asarray(source_points_xyz, dtype=np.float64)
        source_cloud, _ = small_gicp.preprocess_points(
            source[:, :3],
            downsampling_resolution=self.config.scan_voxel_size,
            num_neighbors=self.config.num_neighbors,
            num_threads=self.config.num_threads,
        )
        init = np.asarray(init_T_map_source, dtype=np.float64)
        if self.voxelmap is not None:
            result = small_gicp.align(
                self.voxelmap,
                source_cloud,
                init_T_target_source=init,
                max_correspondence_distance=max_correspondence_distance,
                num_threads=self.config.num_threads,
                max_iterations=self.config.max_iterations,
            )
        else:
            result = small_gicp.align(
                self.cloud,
                source_cloud,
                self.tree,
                init_T_target_source=init,
                registration_type=self.config.registration_type,
                max_correspondence_distance=max_correspondence_distance,
                num_threads=self.config.num_threads,
                max_iterations=self.config.max_iterations,
            )
        return RegistrationOutcome(
            T_map_odom=np.asarray(result.T_target_source),
            converged=bool(result.converged),
            num_inliers=int(result.num_inliers),
            source_size=int(source_cloud.size()),
            error=float(result.error),
        )
