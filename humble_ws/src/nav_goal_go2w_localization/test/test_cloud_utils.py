import numpy as np
import pytest

from nav_goal_go2w_localization.cloud_utils import crop_range, pointcloud2_to_xyz


def test_crop_range():
    points = np.array(
        [[0.1, 0, 0], [1, 0, 0], [5, 0, 0], [40, 0, 0]], dtype=np.float32
    )
    out = crop_range(points, np.zeros(3), 0.5, 30.0)
    np.testing.assert_allclose(out[:, 0], [1, 5])


def test_crop_range_empty():
    out = crop_range(np.empty((0, 3), np.float32), np.zeros(3), 0.5, 30.0)
    assert len(out) == 0


def test_pointcloud2_to_xyz_with_padding_and_nans():
    sensor_msgs = pytest.importorskip("sensor_msgs.msg")
    std_msgs = pytest.importorskip("std_msgs.msg")
    from sensor_msgs.msg import PointCloud2, PointField

    # 4 points with an extra intensity field and trailing padding (step 20).
    raw = np.zeros(4, dtype={
        "names": ["x", "y", "z", "intensity"],
        "formats": [np.float32] * 4,
        "offsets": [0, 4, 8, 12],
        "itemsize": 20,
    })
    raw["x"] = [1, 2, np.nan, 4]
    raw["y"] = [0, 0, 0, 0]
    raw["z"] = [5, 6, 7, 8]
    raw["intensity"] = [9, 9, 9, 9]

    msg = PointCloud2()
    msg.header = std_msgs.Header()
    msg.height = 1
    msg.width = 4
    msg.point_step = 20
    msg.row_step = 80
    msg.fields = [
        PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
        for n, o in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))
    ]
    msg.data = raw.tobytes()

    points = pointcloud2_to_xyz(msg)
    assert points.shape == (3, 3)  # NaN row dropped
    np.testing.assert_allclose(points[:, 0], [1, 2, 4])
    np.testing.assert_allclose(points[:, 2], [5, 6, 8])


def test_pointcloud2_missing_field_raises():
    pytest.importorskip("sensor_msgs.msg")
    from sensor_msgs.msg import PointCloud2, PointField

    msg = PointCloud2()
    msg.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1)]
    with pytest.raises(ValueError):
        pointcloud2_to_xyz(msg)
