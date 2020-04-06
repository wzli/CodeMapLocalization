from geometry_msgs.msg import PoseStamped
from code_map_localization_msgs.msg import Localization
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
