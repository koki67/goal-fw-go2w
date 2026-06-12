"""Atomic finalization of a live D-LIO map into navigation artifacts."""
from __future__ import annotations

import os
import shutil
import tempfile
from pathlib import Path
from typing import Callable

SaveFn = Callable[[Path, float], None]
StatusFn = Callable[[str], None]


def finish_map(
    output: str | Path,
    save_leaf_size: float,
    save_fn: SaveFn,
    status_cb: StatusFn = lambda _status: None,
) -> Path:
    """Save D-LIO, convert it, and atomically publish the completed directory."""
    output_path = Path(output)
    staging_root: Path | None = None
    raw_pcd: Path | None = None
    try:
        if not output_path.is_absolute():
            raise ValueError(f"output must be an absolute path: {output_path}")
        if output_path.exists():
            raise FileExistsError(
                f"output already exists; refusing to overwrite: {output_path}"
            )
        if save_leaf_size <= 0.0:
            raise ValueError("save_leaf_size must be positive")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        staging_root = Path(
            tempfile.mkdtemp(
                prefix=f".{output_path.name}.prepare-map.", dir=output_path.parent
            )
        )
        staging_dir = staging_root / output_path.name
        raw_dir = staging_dir / "raw"
        raw_dir.mkdir(parents=True)
        raw_pcd = raw_dir / "dlio_map.pcd"

        status_cb("SAVING")
        save_fn(raw_dir, float(save_leaf_size))
        if not raw_pcd.is_file() or raw_pcd.stat().st_size == 0:
            raise RuntimeError(f"D-LIO did not create a non-empty {raw_pcd}")

        status_cb("CONVERTING")
        from nav_goal_go2w_map import prepare_map_cli

        result = prepare_map_cli.main(
            ["--input", str(raw_pcd), "--output", str(staging_dir)]
        )
        if result != 0:
            raise RuntimeError(f"prepare_map failed with status {result}")
        for artifact in ("map.pcd", "viz.pcd", "grid.yaml"):
            if not (staging_dir / artifact).is_file():
                raise RuntimeError(f"prepare_map did not create {artifact}")
        if output_path.exists():
            raise FileExistsError(
                f"output appeared during conversion; refusing to overwrite: {output_path}"
            )

        os.rename(staging_dir, output_path)
        staging_root.rmdir()
        status_cb(f"DONE {output_path}")
        return output_path
    except Exception as exc:
        detail = str(exc)
        if staging_root is not None and staging_root.exists():
            # The staged raw PCD may be the only copy of the mapping session,
            # so keep it and tell the operator where it is; an empty staging
            # tree is worthless and would pile up across retries.
            if raw_pcd is not None and raw_pcd.is_file() and raw_pcd.stat().st_size > 0:
                detail += f" (partial map retained in {staging_root})"
            else:
                shutil.rmtree(staging_root, ignore_errors=True)
        status_cb(f"FAILED {detail}")
        raise
