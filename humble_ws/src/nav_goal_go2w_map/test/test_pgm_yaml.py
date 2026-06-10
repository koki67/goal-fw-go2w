import numpy as np
import yaml

from nav_goal_go2w_map.grid_builder_core import BuiltGrid
from nav_goal_go2w_map.pgm_yaml import grid_to_pgm_bytes, write_map_files


def test_pgm_encoding_and_row_flip():
    grid = np.array([[0, 100], [-1, 0]], dtype=np.int8)  # row 0 = south
    raw = grid_to_pgm_bytes(grid)
    header, payload = raw.split(b"255\n", 1)
    assert header == b"P5\n2 2\n"
    pixels = np.frombuffer(payload, dtype=np.uint8).reshape(2, 2)
    # Top image row must be the NORTH grid row (grid row 1).
    assert pixels[0, 0] == 205   # unknown
    assert pixels[0, 1] == 254   # free
    assert pixels[1, 0] == 254   # free
    assert pixels[1, 1] == 0     # occupied


def test_write_map_files_yaml_contract(tmp_path):
    grid = BuiltGrid(
        data=np.zeros((3, 4), dtype=np.int8),
        resolution=0.05,
        origin_x=-1.0,
        origin_y=2.0,
    )
    pgm_path, yaml_path = write_map_files(tmp_path, grid)
    assert pgm_path.exists() and yaml_path.exists()
    meta = yaml.safe_load(yaml_path.read_text())
    assert meta["image"] == "grid.pgm"
    assert meta["mode"] == "trinary"
    assert meta["resolution"] == 0.05
    assert meta["origin"] == [-1.0, 2.0, 0.0]
    assert meta["negate"] == 0
    # Gray (205) must classify as unknown: shade 50/255 = 0.19608 must fall
    # in [free_thresh, occupied_thresh).
    shade_unknown = (255 - 205) / 255.0
    assert meta["free_thresh"] <= shade_unknown < meta["occupied_thresh"]


def test_map_server_threshold_classification():
    # Replicates nav2 map_io trinary classification for all three pixel values.
    free_thresh, occupied_thresh = 0.196, 0.65
    for pixel, expected in ((254, 0), (205, -1), (0, 100)):
        shade = (255 - pixel) / 255.0
        if shade > occupied_thresh:
            value = 100
        elif shade < free_thresh:
            value = 0
        else:
            value = -1
        assert value == expected, f"pixel {pixel}"
