import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from codemap.serial import CsvRxStream
import math


class CodeMapLocalization(Node):
    def __init__(self):
        super().__init__('code_map_localization')
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose', 1)
        serial_device = self.declare_parameter("serial_device")
        csv_file = self.declare_parameter("csv_file")
        frame_id = self.declare_parameter("frame_id")
        self.rx_stream = CsvRxStream(serial_device._value, csv_file._value)
        self.get_logger().info(
            f'Serial Rx started on {serial_device._value} logging to {csv_file._value}'
        )
        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = frame_id._value
        self.timer = self.create_timer(0.02, self.rx_timer_callback)

    def rx_timer_callback(self):
        loc_msg = self.rx_stream.read_message()
        if loc_msg is not None:
            self.pose_msg.header.stamp.sec = loc_msg['timestamp'] >> 10
            self.pose_msg.header.stamp.nanosec = (loc_msg['timestamp']
                                                  & 0x3FF) << 20
            self.pose_msg.pose.position.x = float(loc_msg['odometry']['x'])
            self.pose_msg.pose.position.y = float(loc_msg['odometry']['y'])
            self.pose_msg.pose.orientation.w = math.cos(
                0.5 * loc_msg['odometry']['rotation'])
            self.pose_msg.pose.orientation.z = math.sin(
                0.5 * loc_msg['odometry']['rotation'])
            self.pose_publisher.publish(self.pose_msg)


def main(args=None):
    rclpy.init(args=args)
    code_map_localization = CodeMapLocalization()
    rclpy.spin(code_map_localization)


if __name__ == '__main__':
    main()
