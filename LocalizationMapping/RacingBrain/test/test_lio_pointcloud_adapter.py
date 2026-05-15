import array

import numpy as np
import pytest

pytest.importorskip("sensor_msgs")
pytest.importorskip("sensor_msgs_py")
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from racingbrain.localization.lio_pointcloud_adapter import (
    VELODYNE_POINT_STEP,
    VelodynePointCloudAdapter,
)


def make_cloud(points):
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="timestamp", offset=20, datatype=PointField.UINT32, count=1),
    ]
    dtype = np.dtype(
        {
            "names": ["x", "y", "z", "intensity", "ring", "timestamp"],
            "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<u4"],
            "offsets": [0, 4, 8, 12, 16, 20],
            "itemsize": 24,
        }
    )
    arr = np.zeros(len(points), dtype=dtype)
    for index, point in enumerate(points):
        arr[index] = point

    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = "hesai"
    msg.height = 1
    msg.width = len(arr)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 24
    msg.row_step = msg.point_step * len(arr)
    msg.is_dense = True
    msg.data = array.array("B", arr.tobytes())
    return msg


def test_adapter_outputs_lio_sam_velodyne_layout():
    adapter = VelodynePointCloudAdapter(output_frame_id="lidar_link")
    msg = make_cloud(
        [
            (1.0, 0.0, 0.1, 7.0, 3, 100000000),
            (0.0, 1.0, 0.2, 9.0, 4, 150000000),
        ]
    )

    adapted, stats = adapter.adapt(msg)

    assert adapted is not None
    assert adapted.header.frame_id == "lidar_link"
    assert adapted.point_step == VELODYNE_POINT_STEP
    assert adapted.width == 2
    assert [field.name for field in adapted.fields] == ["x", "y", "z", "intensity", "ring", "time"]
    assert stats.ring_source == "ring"
    assert stats.time_source == "timestamp"


def test_adapter_can_infer_missing_ring_and_time():
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4", "<f4", "<f4"], "offsets": [0, 4, 8], "itemsize": 12})
    arr = np.array([(1.0, 0.0, -0.2), (0.0, 1.0, 0.2)], dtype=dtype)
    msg = PointCloud2()
    msg.header = Header()
    msg.height = 1
    msg.width = len(arr)
    msg.fields = fields
    msg.point_step = 12
    msg.row_step = 24
    msg.data = array.array("B", arr.tobytes())

    adapted, stats = VelodynePointCloudAdapter(output_frame_id="").adapt(msg)

    assert adapted is not None
    assert adapted.width == 2
    assert stats.ring_source == "vertical_angle"
    assert stats.time_source == "azimuth"
