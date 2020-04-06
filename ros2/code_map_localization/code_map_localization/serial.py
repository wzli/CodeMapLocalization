import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from code_map_localization_msgs.msg import Localization
from codemap.serial import CsvRxStream
import math


def copy_attributes(obj, dic):
    for key, value in dic.items():
        if isinstance(value, dict):
            copy_attributes(getattr(obj, key), value)
        else:
            setattr(obj, key, type(getattr(obj, key))(value))


def convert_to_ros_msgs(loc_msg):
    pose_msg = PoseStamped()
    pose_msg.header.stamp.sec = loc_msg['timestamp'] >> 10
    pose_msg.header.stamp.nanosec = (loc_msg['timestamp'] & 0x3FF) << 20
    pose_msg.pose.position.x = float(loc_msg['odometry']['x'])
    pose_msg.pose.position.y = float(loc_msg['odometry']['y'])
    pose_msg.pose.orientation.w = math.cos(0.5 *
                                           loc_msg['odometry']['rotation'])
    pose_msg.pose.orientation.z = math.sin(0.5 *
                                           loc_msg['odometry']['rotation'])
    localization_msg = Localization()
    copy_attributes(localization_msg, loc_msg)
    return pose_msg, localization_msg


class CodeMapLocalization(Node):
    def __init__(self):
        super().__init__('code_map_localization')
        # create publisher
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose', 1)
        self.localization_publisher = self.create_publisher(
            Localization, 'localization', 1)
        # parse serial device
        serial_device = self.declare_parameter("serial_device")
        serial_device = serial_device._value if serial_device else '/dev/ttyUSB0'
        # parse csv file
        csv_file = self.declare_parameter("csv_file")
        if csv_file:
            csv_file = csv_file.value
        # parse frame_id
        self.frame_id = self.declare_parameter("frame_id")
        self.frame_id = self.frame_id._value if self.frame_id else 'map'
        # ceate rx stream
        self.rx_stream = CsvRxStream(serial_device, csv_file)
        # print start message
        self.get_logger().info(
            f'Serial Rx started on {serial_device} logging to {csv_file}')
        # poll every 10ms
        self.timer = self.create_timer(0.01, self.rx_timer_callback)

    def rx_timer_callback(self):
        loc_msg = self.rx_stream.read_message()
        if loc_msg is not None:
            pose_msg, localization_msg = convert_to_ros_msgs(loc_msg)
            pose_msg.header.frame_id = self.frame_id
            self.pose_publisher.publish(pose_msg)
            self.localization_publisher.publish(localization_msg)


def main(args=None):
    rclpy.init(args=args)
    code_map_localization = CodeMapLocalization()
    rclpy.spin(code_map_localization)


if __name__ == '__main__':
    main()
