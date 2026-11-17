import importlib
import math
import struct
import sys
from array import array
from pathlib import Path

import pytest
from sensor_msgs.msg import PointCloud2, PointField


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
add_zero_relative_time = importlib.import_module(
    'pointcloud2_timing_adapter'
).add_zero_relative_time


def _make_cloud(
    width=2,
    height=1,
    point_step=32,
    row_padding=0,
    is_bigendian=False,
    coordinates=None,
):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=24, datatype=PointField.UINT16, count=1),
    ]
    row_step = width * point_step + row_padding
    payload = bytearray(
        (index * 37 + 19) % 256 for index in range(row_step * height)
    )
    endian = '>' if is_bigendian else '<'
    for row in range(height):
        for column in range(width):
            point_index = row * width + column
            point_start = row * row_step + column * point_step
            xyz = (coordinates or {}).get(point_index, (point_index + 1.0, 2.0, 3.0))
            struct.pack_into(endian + 'fff', payload, point_start, *xyz)
            struct.pack_into(
                endian + 'f', payload, point_start + 16, 11.5 + point_index
            )
            struct.pack_into(endian + 'H', payload, point_start + 24, 100 + point_index)
        if row_padding:
            padding_start = row * row_step + width * point_step
            payload[padding_start:(row + 1) * row_step] = bytes(
                [0xD0 + row] * row_padding
            )

    cloud = PointCloud2()
    cloud.header.stamp.sec = 123
    cloud.header.stamp.nanosec = 456
    cloud.header.frame_id = 'helios_16p_link'
    cloud.height = height
    cloud.width = width
    cloud.fields = fields
    cloud.is_bigendian = is_bigendian
    cloud.point_step = point_step
    cloud.row_step = row_step
    cloud.data = array('B', payload)
    cloud.is_dense = False
    return cloud


def _field_signature(field):
    return (field.name, field.offset, field.datatype, field.count)


def _point_start(cloud, row, column):
    return row * cloud.row_step + column * cloud.point_step


@pytest.mark.parametrize('is_bigendian', [False, True])
def test_adds_time_and_preserves_organized_cloud_bytes(is_bigendian):
    cloud = _make_cloud(
        width=2,
        height=2,
        row_padding=4,
        is_bigendian=is_bigendian,
        coordinates={1: (math.nan, math.inf, -math.inf)},
    )
    original_data = bytes(cloud.data)
    original_header = (
        cloud.header.stamp.sec,
        cloud.header.stamp.nanosec,
        cloud.header.frame_id,
    )
    original_fields = [_field_signature(field) for field in cloud.fields]

    adapted = add_zero_relative_time(cloud)

    assert (adapted.width, adapted.height) == (2, 2)
    assert adapted.point_step == 36
    assert adapted.row_step == 76
    assert adapted.is_bigendian is is_bigendian
    assert adapted.is_dense is cloud.is_dense
    assert (
        adapted.header.stamp.sec,
        adapted.header.stamp.nanosec,
        adapted.header.frame_id,
    ) == original_header
    assert [_field_signature(field) for field in adapted.fields[:-1]] == original_fields
    assert _field_signature(adapted.fields[-1]) == (
        'time',
        32,
        PointField.FLOAT32,
        1,
    )
    assert len(adapted.data) == adapted.row_step * adapted.height
    assert bytes(cloud.data) == original_data

    endian = '>' if is_bigendian else '<'
    for row in range(cloud.height):
        for column in range(cloud.width):
            source_start = _point_start(cloud, row, column)
            output_start = _point_start(adapted, row, column)
            assert bytes(adapted.data[output_start:output_start + cloud.point_step]) == (
                original_data[source_start:source_start + cloud.point_step]
            )
            assert struct.unpack_from(endian + 'f', adapted.data, output_start + 32)[0] == 0.0
        source_padding = original_data[
            row * cloud.row_step + cloud.width * cloud.point_step:
            (row + 1) * cloud.row_step
        ]
        output_padding = bytes(adapted.data[
            row * adapted.row_step + adapted.width * adapted.point_step:
            (row + 1) * adapted.row_step
        ])
        assert output_padding == source_padding

    nan_point = _point_start(adapted, 0, 1)
    assert math.isnan(struct.unpack_from(endian + 'f', adapted.data, nan_point)[0])
    assert math.isinf(struct.unpack_from(endian + 'f', adapted.data, nan_point + 4)[0])
    assert struct.unpack_from(endian + 'f', adapted.data, nan_point + 16)[0] == 12.5
    assert struct.unpack_from(endian + 'H', adapted.data, nan_point + 24)[0] == 101


def test_adds_time_to_unorganized_cloud():
    cloud = _make_cloud(width=3, height=1)

    adapted = add_zero_relative_time(cloud)

    assert adapted.width == 3
    assert adapted.height == 1
    assert adapted.row_step == adapted.width * adapted.point_step
    assert adapted.point_step == 36
    for column in range(adapted.width):
        point_start = _point_start(adapted, 0, column)
        assert struct.unpack_from('<f', adapted.data, point_start + 32)[0] == 0.0


def test_aligns_time_field_and_preserves_existing_point_padding():
    cloud = _make_cloud(width=1, point_step=33)
    original_point = bytes(cloud.data)

    adapted = add_zero_relative_time(cloud)

    assert adapted.fields[-1].offset == 36
    assert adapted.point_step == 40
    assert bytes(adapted.data[:33]) == original_point
    assert bytes(adapted.data[33:36]) == b'\x00\x00\x00'
    assert struct.unpack_from('<f', adapted.data, 36)[0] == 0.0


def test_empty_cloud_has_valid_adapted_layout():
    cloud = _make_cloud(width=0, height=1)

    adapted = add_zero_relative_time(cloud)

    assert adapted.width == 0
    assert adapted.height == 1
    assert adapted.point_step == 36
    assert adapted.row_step == 0
    assert len(adapted.data) == 0
    assert adapted.fields[-1].name == 'time'


def test_rejects_existing_point_time_field():
    cloud = _make_cloud()
    cloud.fields.append(
        PointField(name='t', offset=28, datatype=PointField.FLOAT32, count=1)
    )

    with pytest.raises(ValueError, match='already contains'):
        add_zero_relative_time(cloud)


@pytest.mark.parametrize(
    ('row_step', 'data_length', 'message'),
    [
        (63, 63, 'row_step'),
        (64, 63, 'data length'),
    ],
)
def test_rejects_inconsistent_payload(row_step, data_length, message):
    cloud = _make_cloud(width=2, height=1)
    cloud.row_step = row_step
    cloud.data = array('B', bytes(data_length))

    with pytest.raises(ValueError, match=message):
        add_zero_relative_time(cloud)
