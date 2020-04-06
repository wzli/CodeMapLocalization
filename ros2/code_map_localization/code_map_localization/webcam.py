import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from code_map_localization_msgs.msg import Localization
from .convert_message import convert_to_ros_msgs
from codemap.webcam import WebCamLocalization
import ctypes
import time

libcodemap = ctypes.cdll.LoadLibrary('libcodemap.so')


class CodeMapLocalizationWebcam(Node):
    def __init__(self):
        super().__init__('code_map_localization')
        # create publisher
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose', 1)
        self.localization_publisher = self.create_publisher(
            Localization, 'localization', 1)
        # parse serial device
        capture_device = self.declare_parameter("capture_device")
        capture_device = capture_device._value if capture_device else 0
        # parse frame_id
        self.frame_id = self.declare_parameter("frame_id")
        self.frame_id = self.frame_id._value if self.frame_id else 'map'
        # ceate webcam stream
        self.webcam_stream = WebCamLocalization(libcodemap, capture_device)
        # print start message
        self.get_logger().info(
            f'Webcam Localization started on capture device {capture_device}')
        # poll every 10ms
        self.timer = self.create_timer(0.03, self.rx_timer_callback)
        self.start_time = time.time()

    def rx_timer_callback(self):
        loc_msg = self.webcam_stream.update()
        if loc_msg is not None:
            loc_msg['timestamp'] = int((time.time() - self.start_time) * 1000)
            pose_msg, localization_msg = convert_to_ros_msgs(loc_msg)
            pose_msg.header.frame_id = self.frame_id
            self.pose_publisher.publish(pose_msg)
            self.localization_publisher.publish(localization_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CodeMapLocalizationWebcam()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
