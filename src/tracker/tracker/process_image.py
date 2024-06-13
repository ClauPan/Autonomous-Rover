import cv2
import numpy as np;


def find_circles(image, tuning_params):
    blur = 5

    search_window = [tuning_params["x"], tuning_params["y"], tuning_params["width"], tuning_params["height"]]

    working_image    = cv2.blur(image, (blur, blur))

    if search_window is None: 
        search_window = [0.0, 0.0, 1.0, 1.0]

    search_window_px = convert_rect_perc_to_pixels(search_window, image)
    
    
    #- Convert image from BGR to HSV
    working_image = cv2.cvtColor(working_image, cv2.COLOR_BGR2HSV)    
    
    #- Apply HSV threshold
    thresh_min = (tuning_params["min_hue"], tuning_params["min_sat"], tuning_params["min_val"])
    thresh_max = (tuning_params["max_hue"], tuning_params["max_sat"], tuning_params["max_val"])
    working_image = cv2.inRange(working_image, thresh_min, thresh_max)


    # Dilate and Erode
    working_image = cv2.dilate(working_image, None, iterations=2)
    working_image = cv2.erode(working_image, None, iterations=2)
    

    # Make a copy of the image for tuning
    tuning_image = cv2.bitwise_and(image,image, mask=working_image)

    # Apply the search window
    working_image = apply_search_window(working_image, search_window)

    # Invert the image to suit the blob detector
    working_image = 255 - working_image



    # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
        
    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 100
        
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 30
    params.maxArea = 20000
        
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
        
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5
        
    # Filter by Inertia
    params.filterByInertia =True
    params.minInertiaRatio = 0.5

    detector = cv2.SimpleBlobDetector_create(params)

    # Run detection!
    keypoints = detector.detect(working_image)

    size_min_px = tuning_params['min_size'] * working_image.shape[1] / 100.0
    size_max_px = tuning_params['max_size'] * working_image.shape[1] / 100.0

    keypoints = [k for k in keypoints if k.size > size_min_px and k.size < size_max_px]

    
    # Set up main output image
    line_color=(0,0,255)

    out_image = cv2.drawKeypoints(image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    out_image = draw_window2(out_image, search_window_px)

    # Set up tuning output image
    
    tuning_image = cv2.drawKeypoints(tuning_image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # tuning_image = draw_window(tuning_image, search_window)
    # cv2.rectangle(image,(x_min_px,y_min_px),(x_max_px,y_max_px),color,line)
    tuning_image = draw_window2(tuning_image, search_window_px)


    keypoints_normalised = [normalise_keypoint(working_image, k) for k in keypoints]

    return keypoints_normalised, out_image, tuning_image


def apply_search_window(image, window=[0.0, 0.0, 1.0, 1.0]):
    x = int(image.shape[1] * window[0] / 100)
    y = int(image.shape[0] * window[1] / 100)
    width = int(image.shape[1] * window[2] / 100)
    height = int(image.shape[0] * window[3] / 100)

    mask = np.zeros(image.shape,np.uint8)
    mask[y:height, x:width] = image[y:height, x:width]   
    
    return(mask)


def draw_window2(image, rect_px, color=(255,0,0), line=5):
    return cv2.rectangle(image,(rect_px[0],rect_px[1]),(rect_px[2],rect_px[3]),color,line)


def convert_rect_perc_to_pixels(rect_perc, image):
    return [int(a*b/100) for a,b in zip(rect_perc, [image.shape[1], image.shape[0], image.shape[1], image.shape[0]])]


def normalise_keypoint(cv_image, kp):
    center_x = 0.5 * float(cv_image.shape[1])
    center_y = 0.5 * float(cv_image.shape[0])

    x = (kp.pt[0] - center_x)/(center_x)
    y = (kp.pt[1] - center_y)/(center_y)

    return cv2.KeyPoint(x, y, kp.size/cv_image.shape[1])


def create_tuning_window(calibartion_params):
    cv2.namedWindow("Calibration Parameters", 0)
    cv2.createTrackbar("x",         "Calibration Parameters", calibartion_params['x'],         100, no_op)
    cv2.createTrackbar("y",         "Calibration Parameters", calibartion_params['y'],         100, no_op)
    cv2.createTrackbar("width",     "Calibration Parameters", calibartion_params['width'],     100, no_op)
    cv2.createTrackbar("height",    "Calibration Parameters", calibartion_params['height'],    100, no_op)
    cv2.createTrackbar("min_size",  "Calibration Parameters", calibartion_params['min_size'],  100, no_op)
    cv2.createTrackbar("max_size",  "Calibration Parameters", calibartion_params['max_size'],  100, no_op)
    cv2.createTrackbar("min_hue",   "Calibration Parameters", calibartion_params['min_hue'],   180, no_op)
    cv2.createTrackbar("max_hue",   "Calibration Parameters", calibartion_params['max_hue'],   180, no_op)
    cv2.createTrackbar("min_sat",   "Calibration Parameters", calibartion_params['min_sat'],   255, no_op)
    cv2.createTrackbar("max_sat",   "Calibration Parameters", calibartion_params['max_sat'],   255, no_op)
    cv2.createTrackbar("min_val",   "Calibration Parameters", calibartion_params['min_val'],   255, no_op)
    cv2.createTrackbar("max_val",   "Calibration Parameters", calibartion_params['max_val'],   255, no_op)


def get_tuning_params():
    return {key:cv2.getTrackbarPos(key, "Calibration Parameters") for key in ["x", "y", "width", "height", "min_size", "max_size", "min_hue", "max_hue", "min_sat", "max_sat", "min_val", "max_val"]}


def wait_on_gui():
    cv2.waitKey(2)


def no_op(x):
    pass