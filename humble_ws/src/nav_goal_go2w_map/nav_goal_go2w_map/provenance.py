"""Stable provenance paths for prepared map metadata."""

from pathlib import Path


def metadata_source(input_path: str | Path, output_dir: str | Path) -> str:
    """Return an absolute source, or a relative path for retained raw data."""
    resolved_input = Path(input_path).resolve()
    resolved_output = Path(output_dir).resolve()
    try:
        return str(resolved_input.relative_to(resolved_output))
    except ValueError:
        return str(resolved_input)
