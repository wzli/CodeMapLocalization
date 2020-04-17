import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from code_map_localization_msgs.msg import Localization
from .convert_message import convert_to_ros_msgs
from codemap.serial import CsvRxStream


class CodeMapLocalizationSerial(Node):
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
    node = CodeMapLocalizationSerial()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
