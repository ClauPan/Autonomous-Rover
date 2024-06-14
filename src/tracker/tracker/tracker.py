import cv2
import time
import rclpy
import random
import numpy as np;
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class Model:
    def __init__(self, model_type):
        self.model_type = model_type

        self.net = self.load()

    def load(self):
        if self.model_type == "ssd":
            pass

        elif self.model_type == "yolo":
            pass

        else:
            params = cv2.SimpleBlobDetector_Params()
            
            params.minThreshold = 0
            params.maxThreshold = 100
            params.filterByArea = True
            params.minArea = 30
            params.maxArea = 20000
            params.filterByCircularity = True
            params.minCircularity = 0.1
            params.filterByConvexity = True
            params.minConvexity = 0.5
            params.filterByInertia =True
            params.minInertiaRatio = 0.5

            return cv2.SimpleBlobDetector_create(params) 


class CalibrationParameters:
    def __init__(self, x, y, width, height, min_size, max_size, min_hue, max_hue, min_sat, max_sat, min_val, max_val, experimenting, calibrating):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.min_size = min_size
        self.max_size = max_size
        self.min_hue = min_hue
        self.max_hue = max_hue
        self.min_sat = min_sat
        self.max_sat = max_sat
        self.min_val = min_val
        self.max_val = max_val

        self.experimenting = experimenting
        self.calibrating = calibrating

    def read_from_gui(self):
        self.x = cv2.getTrackbarPos("x", "Calibration Parameters")
        self.y = cv2.getTrackbarPos("y", "Calibration Parameters")
        self.width = cv2.getTrackbarPos("width", "Calibration Parameters")
        self.height = cv2.getTrackbarPos("height", "Calibration Parameters")
        self.min_size = cv2.getTrackbarPos("min_size", "Calibration Parameters")
        self.max_size = cv2.getTrackbarPos("max_size", "Calibration Parameters")
        self.min_hue = cv2.getTrackbarPos("min_hue", "Calibration Parameters")
        self.max_hue = cv2.getTrackbarPos("max_hue", "Calibration Parameters")
        self.min_sat = cv2.getTrackbarPos("min_sat", "Calibration Parameters")
        self.max_sat = cv2.getTrackbarPos("max_sat", "Calibration Parameters")
        self.min_val = cv2.getTrackbarPos("min_val", "Calibration Parameters")
        self.max_val = cv2.getTrackbarPos("max_val", "Calibration Parameters")


class Tracker(Node):
    def __init__(self):
        super().__init__('tracker')

        self.get_logger().info('Initializing...')

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
        self.declare_parameter('experiment_mode', False)
        self.declare_parameter('calibration_mode', False)

        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.7)
        self.declare_parameter("forward_chase_speed", 0.1)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)
        
        self.calibration_params = CalibrationParameters(
            self.get_parameter('x').get_parameter_value().integer_value,
            self.get_parameter('y').get_parameter_value().integer_value,
            self.get_parameter('width').get_parameter_value().integer_value,
            self.get_parameter('height').get_parameter_value().integer_value,
            self.get_parameter('min_size').get_parameter_value().integer_value,
            self.get_parameter('max_size').get_parameter_value().integer_value,
            self.get_parameter('min_hue').get_parameter_value().integer_value,
            self.get_parameter('max_hue').get_parameter_value().integer_value,
            self.get_parameter('min_sat').get_parameter_value().integer_value,
            self.get_parameter('max_sat').get_parameter_value().integer_value,
            self.get_parameter('min_val').get_parameter_value().integer_value,
            self.get_parameter('max_val').get_parameter_value().integer_value,
            self.get_parameter('experiment_mode').get_parameter_value().bool_value,
            self.get_parameter('calibration_mode').get_parameter_value().bool_value
        )

        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value


        self.camera_feed_cb_group = MutuallyExclusiveCallbackGroup()
        self.follower_cb_group = MutuallyExclusiveCallbackGroup()

        camera_feed_topic = "/camera/image_raw"
        if self.calibration_params.calibrating:
            camera_feed_topic = "/camera/image_raw/uncompressed"

        self.camera_feed = self.create_subscription(Image, camera_feed_topic, self.detect, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value, callback_group=self.camera_feed_cb_group)
        if not self.calibration_params.experimenting:
            self.follower = self.create_timer(0.1, self.follow, callback_group=self.follower_cb_group)

        self.image_out_pub = self.create_publisher(Image, "/detection_overlay", 1)
        self.image_cal_pub = self.create_publisher(Image, "/camera_calibration", 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_tracker', 10)

        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000
        self.lost_ct = 0

        self.blank_frames = []
        self.prev_eta = -1
        self.eta = -1
        self.countdown = 5
        self.max_frames = 100
        self.switch_threshold = 20
        self.frame_count = 0

        self.model = Model("default")

        self.bridge = CvBridge()

        if(self.calibration_params.calibrating):
            create_tuning_window(self.calibration_params)

    def detect(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

            if self.calibration_params.calibrating:
                self.calibration_params.read_from_gui() 

            if self.calibration_params.experimenting:
                if len(self.blank_frames) < 30:
                    if len(self.blank_frames) == 0:
                        self.get_logger().info("Staring tracker in 'EXPERIMENT_MODE'")
                        self.get_logger().info("Gathering blank frames...")
                    self.blank_frames.append(frame)
                    if len(self.blank_frames) == 30:
                        self.get_logger().info("Blank frames acquired. Place the object in the center of the camera")
                else:
                    if self.frame_count == self.max_frames:
                        raise SystemExit

                    if self.eta == -1:
                        self.eta = time.time()

                    eta = time.time() - self.eta
                    if eta > self.countdown:
                        if int(self.frame_count / self.switch_threshold) % 2 == 0:
                            frame = random.choice(self.blank_frames)

                        self.frame_count += 1


                    else:
                        if int(eta) > self.prev_eta:
                            self.prev_eta = int(eta)
                            self.get_logger().info(f"Starting experiment in {10 - self.prev_eta}...")
                        frame = random.choice(self.blank_frames)




            keypoints_norm, out_image, tuning_image = detect_objects(frame, self.model, self.calibration_params)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = data.header
            self.image_cal_pub.publish(img_to_pub)

            point_out = Point()

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
                self.target_val = self.target_val * self.filter_value + point_out.x * (1-self.filter_value)
                self.target_dist = self.target_dist * self.filter_value + point_out.z * (1-self.filter_value)
                self.lastrcvtime = time.time() 

        except CvBridgeError as e:
            print(e)  

    def follow(self):
        msg = Twist()
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
            self.lost_ct = 0
            self.get_logger().info('Target: {}'.format(self.target_val))
            print(self.target_dist)
            if (self.target_dist < self.max_size_thresh):
                msg.linear.x = self.forward_chase_speed
            msg.angular.z = -self.angular_chase_multiplier*self.target_val
        else:
            if self.lost_ct == 0:
                self.get_logger().info('Target lost...')
            
            self.lost_ct += 1

            if self.lost_ct > 20:
                msg.angular.z = -self.search_angular_speed
        self.cmd_vel_pub.publish(msg)



def run_model_deault(frame, model, calibration_params):
    search_window = [calibration_params.x, calibration_params.y, calibration_params.width, calibration_params.height]

    if search_window is None: 
        search_window = [0.0, 0.0, 1.0, 1.0]

    search_window_px = [int(a*b/100) for a,b in zip(search_window, [frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])]

    filtered_frame = cv2.blur(frame, (5, 5))
    filtered_frame = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2HSV)        
    filtered_frame = cv2.inRange(
        filtered_frame, 
        (calibration_params.min_hue, calibration_params.min_sat, calibration_params.min_val), 
        (calibration_params.max_hue, calibration_params.max_sat, calibration_params.max_val)
    )
    filtered_frame = cv2.dilate(filtered_frame, None, iterations=2)
    filtered_frame = cv2.erode(filtered_frame, None, iterations=2)

    tuning_image = cv2.bitwise_and(frame, frame, mask=filtered_frame)

    x = int(filtered_frame.shape[1] * search_window[0] / 100)
    y = int(filtered_frame.shape[0] * search_window[1] / 100)
    width = int(filtered_frame.shape[1] * search_window[2] / 100)
    height = int(filtered_frame.shape[0] * search_window[3] / 100)

    mask = np.zeros(filtered_frame.shape, np.uint8)
    mask[y:height, x:width] = filtered_frame[y:height, x:width]   

    filtered_frame = 255 - (mask)

    keypoints = model.net.detect(filtered_frame)

    size_min_px = calibration_params.min_size * filtered_frame.shape[1] / 100.0
    size_max_px = calibration_params.max_size * filtered_frame.shape[1] / 100.0

    keypoints = [k for k in keypoints if k.size > size_min_px and k.size < size_max_px]

    
    out_image = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    out_image = cv2.rectangle(out_image, (search_window_px[0], search_window_px[1]), (search_window_px[2], search_window_px[3]), (255, 0, 0), 5)

    tuning_image = cv2.drawKeypoints(tuning_image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    tuning_image = cv2.rectangle(tuning_image, (search_window_px[0], search_window_px[1]), (search_window_px[2], search_window_px[3]), (255, 0, 0), 5)


    keypoints_normalised = [normalise_keypoint(filtered_frame, k) for k in keypoints]

    return keypoints_normalised, out_image, tuning_image


def detect_objects(frame, model, calibration_params):
    if model.model_type == "ssd":
        pass
    elif model.model_type == "yolo":
        pass
    else:
        return run_model_deault(frame, model, calibration_params)

    
def normalise_keypoint(working_image, keypoint):
    center_x = 0.5 * float(working_image.shape[1])
    center_y = 0.5 * float(working_image.shape[0])

    x = (keypoint.pt[0] - center_x)/(center_x)
    y = (keypoint.pt[1] - center_y)/(center_y)

    return cv2.KeyPoint(x, y, keypoint.size/working_image.shape[1])


def nothing(x):
    pass


def create_tuning_window(calibration_params):
    cv2.namedWindow("Calibration Parameters", 0)
    cv2.createTrackbar("x",         "Calibration Parameters", calibration_params.x,         100, nothing)
    cv2.createTrackbar("y",         "Calibration Parameters", calibration_params.y,         100, nothing)
    cv2.createTrackbar("width",     "Calibration Parameters", calibration_params.width,     100, nothing)
    cv2.createTrackbar("height",    "Calibration Parameters", calibration_params.height,    100, nothing)
    cv2.createTrackbar("min_size",  "Calibration Parameters", calibration_params.min_size,  100, nothing)
    cv2.createTrackbar("max_size",  "Calibration Parameters", calibration_params.max_size,  100, nothing)
    cv2.createTrackbar("min_hue",   "Calibration Parameters", calibration_params.min_hue,   180, nothing)
    cv2.createTrackbar("max_hue",   "Calibration Parameters", calibration_params.max_hue,   180, nothing)
    cv2.createTrackbar("min_sat",   "Calibration Parameters", calibration_params.min_sat,   255, nothing)
    cv2.createTrackbar("max_sat",   "Calibration Parameters", calibration_params.max_sat,   255, nothing)
    cv2.createTrackbar("min_val",   "Calibration Parameters", calibration_params.min_val,   255, nothing)
    cv2.createTrackbar("max_val",   "Calibration Parameters", calibration_params.max_val,   255, nothing)


def main(args=None):

    rclpy.init(args=args)

    tracker = Tracker()
    while rclpy.ok():
        try:
            rclpy.spin_once(tracker)
        except SystemExit: 
            rclpy.logging.get_logger("tracker").info('Ending experiment...')
            break
        cv2.waitKey(2)

    tracker.destroy_node()
    rclpy.shutdown()

