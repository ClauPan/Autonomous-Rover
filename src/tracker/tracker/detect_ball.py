import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
import tracker.process_image as proc

class DetectBall(Node):

    def __init__(self):
        super().__init__('detect_ball')

        self.get_logger().info('Looking for the ball...')
        self.image_sub = self.create_subscription(Image,"/image_in",self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)
        self.ball_pub  = self.create_publisher(Point,"/detected_ball",1)

        self.declare_parameter('tuning_mode', False)

        self.declare_parameter("x", 0)
        self.declare_parameter("y", 0)
        self.declare_parameter("width", 100)
        self.declare_parameter("height", 100)
        self.declare_parameter("min_size", 0)
        self.declare_parameter("max_size", 100)
        self.declare_parameter("min_hue", 0)
        self.declare_parameter("max_hue", 180)
        self.declare_parameter("min_sat", 0)
        self.declare_parameter("max_sat", 255)
        self.declare_parameter("min_val", 0)
        self.declare_parameter("max_val", 255)
        
        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x':        self.get_parameter('x').get_parameter_value().integer_value,
            'y':        self.get_parameter('y').get_parameter_value().integer_value,
            'width':    self.get_parameter('width').get_parameter_value().integer_value,
            'height':   self.get_parameter('height').get_parameter_value().integer_value,
            'min_size': self.get_parameter('min_size').get_parameter_value().integer_value,
            'max_size': self.get_parameter('max_size').get_parameter_value().integer_value,
            'min_hue':  self.get_parameter('min_hue').get_parameter_value().integer_value,
            'max_hue':  self.get_parameter('max_hue').get_parameter_value().integer_value,
            'min_sat':  self.get_parameter('min_sat').get_parameter_value().integer_value,
            'max_sat':  self.get_parameter('max_sat').get_parameter_value().integer_value,
            'min_val':  self.get_parameter('min_val').get_parameter_value().integer_value,
            'max_val':  self.get_parameter('max_val').get_parameter_value().integer_value
        }

        self.bridge = CvBridge()

        if(self.tuning_mode):
            proc.create_tuning_window(self.tuning_params)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            if (self.tuning_mode):
                self.tuning_params = proc.get_tuning_params()

            keypoints_norm, out_image, tuning_image = proc.find_circles(cv_image, self.tuning_params)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = data.header
            self.image_tuning_pub.publish(img_to_pub)

            point_out = Point()

            # Keep the biggest point
            # They are already converted to normalised coordinates
            for i, kp in enumerate(keypoints_norm):
                x = kp.pt[0]
                y = kp.pt[1]
                s = kp.size

                self.get_logger().info(f"Pt {i}: ({x},{y},{s})")

                if (s > point_out.z):                    
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s

            if (point_out.z > 0):
                self.ball_pub.publish(point_out) 
        except CvBridgeError as e:
            print(e)  


def main(args=None):

    rclpy.init(args=args)

    detect_ball = DetectBall()
    while rclpy.ok():
        rclpy.spin_once(detect_ball)
        proc.wait_on_gui()

    detect_ball.destroy_node()
    rclpy.shutdown()

