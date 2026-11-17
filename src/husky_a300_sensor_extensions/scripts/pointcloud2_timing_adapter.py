#!/usr/bin/env python3

from array import array
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


DEFAULT_INPUT_TOPIC = '/a300_00000/sensors/helios_16p/points'
DEFAULT_OUTPUT_TOPIC = '/a300_00000/sensors/helios_16p/points_glim'
TIME_FIELD_NAME = 'time'
_UINT32_MAX = (1 << 32) - 1
_TIME_FIELD_NAMES = {'t', 'time', 'time_stamp', 'timestamp'}
_FIELD_SIZES = {
    PointField.INT8: 1,
    PointField.UINT8: 1,
    PointField.INT16: 2,
    PointField.UINT16: 2,
    PointField.INT32: 4,
    PointField.UINT32: 4,
    PointField.FLOAT32: 4,
    PointField.FLOAT64: 8,
}


def add_zero_relative_time(cloud: PointCloud2) -> PointCloud2:
    """Return a byte-preserving copy with FLOAT32 zero-second relative times."""
    names = [field.name for field in cloud.fields]
    if len(names) != len(set(names)):
        raise ValueError('input contains duplicate PointField names')
    if _TIME_FIELD_NAMES.intersection(names):
        raise ValueError('input already contains a recognized point-time field')

    fields_by_name = {field.name: field for field in cloud.fields}
    coordinates = []
    for axis in ('x', 'y', 'z'):
        field = fields_by_name.get(axis)
        if (
            field is None
            or field.datatype not in (PointField.FLOAT32, PointField.FLOAT64)
            or field.count != 1
        ):
            raise ValueError(f'input must contain a floating-point {axis} field')
        coordinates.append(field)
    if len({field.datatype for field in coordinates}) != 1:
        raise ValueError('input x, y, and z fields must use the same datatype')

    for field in cloud.fields:
        field_size = _FIELD_SIZES.get(field.datatype)
        if field_size is None or field.count < 1:
            raise ValueError(f'input field {field.name!r} has an invalid datatype or count')
        if field.offset + field_size * field.count > cloud.point_step:
            raise ValueError(f'input field {field.name!r} extends beyond point_step')

    minimum_row_step = cloud.width * cloud.point_step
    if cloud.row_step < minimum_row_step:
        raise ValueError('input row_step is smaller than width * point_step')

    input_data = bytes(cloud.data)
    if len(input_data) != cloud.row_step * cloud.height:
        raise ValueError('input data length does not equal row_step * height')

    time_offset = (cloud.point_step + 3) & ~3
    output_point_step = time_offset + _FIELD_SIZES[PointField.FLOAT32]
    point_growth = output_point_step - cloud.point_step
    output_row_step = cloud.row_step + cloud.width * point_growth
    output_data_size = output_row_step * cloud.height
    if output_point_step > _UINT32_MAX or output_row_step > _UINT32_MAX:
        raise ValueError('adapted point or row step exceeds the PointCloud2 uint32 limit')
    if output_data_size > _UINT32_MAX:
        raise ValueError('adapted cloud payload exceeds the PointCloud2 sequence limit')

    output_data = bytearray(output_data_size)
    input_row_data_size = cloud.width * cloud.point_step
    output_row_data_size = cloud.width * output_point_step
    for row in range(cloud.height):
        input_row_start = row * cloud.row_step
        output_row_start = row * output_row_step
        for column in range(cloud.width):
            input_point_start = input_row_start + column * cloud.point_step
            output_point_start = output_row_start + column * output_point_step
            output_data[
                output_point_start:output_point_start + cloud.point_step
            ] = input_data[input_point_start:input_point_start + cloud.point_step]
        input_padding_start = input_row_start + input_row_data_size
        output_padding_start = output_row_start + output_row_data_size
        output_data[
            output_padding_start:output_row_start + output_row_step
        ] = input_data[input_padding_start:input_row_start + cloud.row_step]

    output = PointCloud2()
    output.header = deepcopy(cloud.header)
    output.height = cloud.height
    output.width = cloud.width
    output.fields = [deepcopy(field) for field in cloud.fields]
    output.fields.append(
        PointField(
            name=TIME_FIELD_NAME,
            offset=time_offset,
            datatype=PointField.FLOAT32,
            count=1,
        )
    )
    output.is_bigendian = cloud.is_bigendian
    output.point_step = output_point_step
    output.row_step = output_row_step
    output.data = array('B', output_data)
    output.is_dense = cloud.is_dense
    return output


class PointCloud2TimingAdapter(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud2_timing_adapter')
        self.declare_parameter('input_topic', DEFAULT_INPUT_TOPIC)
        self.declare_parameter('output_topic', DEFAULT_OUTPUT_TOPIC)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        if not input_topic or not output_topic:
            raise ValueError('input_topic and output_topic must be non-empty')
        if input_topic == output_topic:
            raise ValueError('input_topic and output_topic must be different')

        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, qos_profile_sensor_data
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        try:
            adapted = add_zero_relative_time(message)
        except ValueError as error:
            self.get_logger().error(f'Rejecting PointCloud2: {error}')
            return
        self.publisher.publish(adapted)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PointCloud2TimingAdapter()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
