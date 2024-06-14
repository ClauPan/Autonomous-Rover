import cv2
import rclpy

from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class camera_interface(Node):

	def __init__(self):
		super().__init__("camera_interface")

		self.camera = cv2.VideoCapture