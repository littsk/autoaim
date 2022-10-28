import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv

class ArmorDetNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.qos = QoSPresetProfiles.SENSOR_DATA
        self.sub = self.create_subscription(Image, "image_raw", self.sub_callback, self.qos)
        self.cv_bridge = CvBridge()

        