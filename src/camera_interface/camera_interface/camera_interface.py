import cv2
import rclpy

from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class camera_interface(Node):

	def __init__(self):
		super().__init__("camera_interface")

		self.camera = cv2.VideoCapture(0)
		self.bridge = CvBridge()

		self.publisher = self.create_publisher(Image, "/camera/image_raw", 20)
		self.timer = self.create_timer(0.1, self.get_frame)

		self.initial_frame = None
		self.frame_counter = 0

	def get_frame(self):
		success, frame = self.camera.read()

		frame = cv2.resize(frame, (380, 240), interpolation=cv2.INTER_CUBIC)
		if success:
			if self.initial_frame is None:
				self.initial_frame = frame

			self.publisher.publish(self.bridgeObject.cv2_to_imgmsg(frame))

			self.frame_counter += 1

def main(args=None):
	rclpy.init(args=args)
	
	interface = camera_interface()

	rclpy.spin(interface)

	interface.destroy_node()

	rclpy.shutdown()

if __name__ == "__main__":
	main()
