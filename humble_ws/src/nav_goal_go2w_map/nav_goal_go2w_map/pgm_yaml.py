"""Write occupancy grids in nav2 map_server format (P5 PGM + YAML)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from nav_goal_go2w_map.grid_builder_core import BuiltGrid, FREE

_PIXEL_FREE = 254
_PIXEL_OCCUPIED = 0
_PIXEL_UNKNOWN = 205
# Thresholds chosen so (255 - pixel) / 255 classifies the three pixel values
# unambiguously: 254 -> 0.004 (< free), 205 -> 0.1961 (between), 0 -> 1.0
# (> occupied). free_thresh must stay below 0.19607 for gray to load unknown.
_FREE_THRESH = 0.196
_OCCUPIED_THRESH = 0.65


def grid_to_pgm_bytes(grid_data: np.ndarray) -> bytes:
    """Encode an int8 occupancy array (-1/0/100, row 0 = south) as P5 PGM."""
    grid_data = np.asarray(grid_data)
    image = np.full(grid_data.shape, _PIXEL_UNKNOWN, dtype=np.uint8)
    image[grid_data == FREE] = _PIXEL_FREE
    image[grid_data >= 65] = _PIXEL_OCCUPIED
    # PGM stores the top image row first; grid row 0 is the south (min y) edge.
    image = np.flipud(image)
    height, width = image.shape
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    return header + image.tobytes()


def write_map_files(output_dir: str | Path, grid: BuiltGrid) -> tuple[Path, Path]:
    """Write grid.pgm + grid.yaml into ``output_dir``; returns their paths."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    pgm_path = out / "grid.pgm"
    yaml_path = out / "grid.yaml"
    pgm_path.write_bytes(grid_to_pgm_bytes(grid.data))
    metadata = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": float(grid.resolution),
        "origin": [float(grid.origin_x), float(grid.origin_y), 0.0],
        "negate": 0,
        "occupied_thresh": _OCCUPIED_THRESH,
        "free_thresh": _FREE_THRESH,
    }
    with yaml_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(metadata, handle, sort_keys=False)
    return pgm_path, yaml_path
