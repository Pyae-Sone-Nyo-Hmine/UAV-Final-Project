import cv2
import cv2.aruco as aruco
import keyboard as kb
from utils.MarkerClass import *

# values for cv2.putText
font = cv2.FONT_HERSHEY_SIMPLEX
org = (10, 50)
font_size = 0.7
color = (255, 0, 0)
thickness = 2

"""FUNCTIONS FOR CONTROLLING DRONE MANUALLY AND WITH AR MARKER"""


# detects the ar marker and gives values of 4 corners and ids
def ar_detector(img, draw_box_id=True, draw_px_val=False):
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    aruco_param = aruco.DetectorParameters_create()
    corners, ids, rejected = aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_param)

    # draw the border and id on the marker
    if draw_box_id:
        aruco.drawDetectedMarkers(img, corners, ids)

    # draw the pixel values of the AR marker
    if draw_px_val:
        try:
            write = str(corners[0][0][0]) + str(corners[0][0][1]) + str(corners[0][0][2]) + str(corners[0][0][3])
        except:
            write = "AR not detected"

        cv2.putText(img, write, org, font, font_size, color, thickness, cv2.LINE_AA)

    return corners, ids


# prints out average length and centroid values from the class Marker
def print_avg_length_centroid(my_ar):
    try:
        my_ar.average_length_func()
        my_ar.centroid_func()
        print("Average length: " + str(my_ar.average_length))
        print("Centroid: " + str(my_ar.centroid))

    except:
        print("AR marker not detected")


# prints out the position and orientation with respect to the ar marker
def print_pos_and_ori(my_ar):
    try:
        my_ar.average_length_func()
        my_ar.centroid_func()
        my_ar.position_func()
        my_ar.orientation_func()
        print(my_ar.position + " " + my_ar.orientation)

    except:
        print("AR marker not Detected")


# control motion using keys from keyboard
def motion_func(my_drone):
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50

    if kb.is_pressed('left'):
        lr = -speed

    elif kb.is_pressed('right'):
        lr = speed

    if kb.is_pressed('up'):
        fb = speed

    elif kb.is_pressed('down'):
        fb = -speed

    if kb.is_pressed('w'):
        ud = speed

    elif kb.is_pressed('s'):
        ud = -speed

    if kb.is_pressed('a'):
        yv = -speed - 20

    elif kb.is_pressed('d'):
        yv = speed + 20

    if kb.is_pressed('q'):
        my_drone.land()

    if kb.is_pressed('e'):
        my_drone.takeoff()

    return [lr, fb, ud, yv]


# autonomously orient the drone with respect to the ar marker
def ar_marker_orientation(my_ar):
    lr, fb, ud, yv = 0, 0, 0, 0
    auto_speed = 30
    auto_yaw = 45

    try:
        my_ar.centroid_func()
        pos_x, pos_y = my_ar.position_func(position_offset=70)

        if pos_x == "Left":
            yv = -auto_yaw

        elif pos_x == "Right":
            yv = auto_yaw

        if pos_y == "Up":
            ud = auto_speed

        elif pos_y == "Down":
            ud = - auto_speed

        else:
            lr, fb, ud, yv = 0, 0, 0, 0

    except:
        pass

    return [lr, fb, ud, yv]


# autonomously orient the drone with respect to the ar marker
# Uses PID control
def ar_marker_up_down_with_PID(my_ar, kp, ki, kd, prev_y_error):
    my_ar.centroid_func()
    y_error = my_ar.centroid[1] - 240

    y_speed = kp * y_error + ki * y_error + kd * (y_error - prev_y_error)
    y_speed = -(int(np.clip(y_speed, -100, 100)))

    return y_error, y_speed


def ar_marker_yaw_with_PID(my_ar, kp, ki, kd, prev_yaw_error):
    my_ar.centroid_func()

    yaw_error = my_ar.centroid[0] - 360
    yaw_speed = kp * yaw_error + ki * yaw_error + kd * (yaw_error - prev_yaw_error)
    yaw_speed = int(np.clip(yaw_speed, -100, 100))

    return yaw_error, yaw_speed


# Follow the ar tag at a respectable distance
# Uses PID control
def follow_ar_with_PID(my_ar, kp, ki, kd, prev_d_error):
    my_ar.average_length_func()

    error = my_ar.average_length - 125
    speed = kp * error + ki * error + kd * (error - prev_d_error)
    speed = -(int(np.clip(speed, -100, 100)))

    return error, speed


def slope_orientation_with_PID(my_ar, kp, ki, kd, prev_s_error):
    my_ar.orientation_func()

    s_error = my_ar.slope_x
    s_speed = kp * s_error + ki * s_error + kd * (s_error - prev_s_error)
    s_speed = int(np.clip(s_speed, -100, 100))

    if abs(my_ar.slope_y) > 0.1:
        s_speed = 0

    return s_error, - s_speed


def traffic_action_with_ar(my_ar, my_drone, ids):
    if my_ar.average_length == 125:
        if ids[0][0] == 0:
            my_drone.rotate_clockwise(90)  # right turn
        elif ids[0][0] == 1:
            my_drone.rotate_counter_clockwise(90)  # left turn
        elif ids[0][0] == 2:
            my_drone.move_up(30)  # move up
        elif ids[0][0] == 3:
            my_drone.move_down(30)  # move down


""" FUNCTIONS FOR COLOR DETECTION"""


def get_colors(img):
    kernel = np.ones((7, 7), np.uint8)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_green = np.array([40, 35, 90])  # adjust as needed
    upper_green = np.array([85, 255, 255])  # adjust as needed
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    res_green = cv2.bitwise_and(img, img, mask=mask_green)
    opening_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    contours_green, _ = cv2.findContours(opening_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    lower_yellow = np.array([20, 35, 90])  # adjust as needed
    upper_yellow = np.array([26, 255, 255])  # adjust as needed
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    res_yellow = cv2.bitwise_and(img, img, mask=mask_yellow)
    opening_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
    contours_yellow, _ = cv2.findContours(opening_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    lower_red1 = np.array([0, 25, 90])  # adjust as needed
    upper_red1 = np.array([8, 255, 255])  # adjust as needed
    lower_red2 = np.array([173, 25, 90])  # adjust as needed
    upper_red2 = np.array([180, 255, 255])  # adjust as needed
    mask1_red = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2_red = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask1_red, mask2_red)
    res_red = cv2.bitwise_and(img, img, mask=mask_red)
    opening_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    contours_red, _ = cv2.findContours(opening_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return contours_green, contours_yellow, contours_red


def find_size(contours):
    if len(contours) == 0:
        return 0

    con_x = list([i[0] for i in contours])[0][0]
    con_y = list([i[1] for i in contours])[0][0]
    #print(con_x)
    #print(con_y)

    max_x = con_x[0]
    min_x = con_x[0]
    #print(max_x)
    #print(min_x)

    max_y = con_y[0]
    min_y = con_y[0]
    #print(max_y)
    #print(min_y)

    for i in con_x:
        if i > max_x:
            max_x = i
        if i < min_x:
            min_x = i

    for i in con_y:
        if i > max_y:
            max_y = i
        if i < min_y:
            min_y = i

    xlen = max_x - min_x
    ylen = max_y - min_y

    return xlen * ylen


def find_largest(green_cont, yellow_cont, red_cont):
    green_area = find_size(green_cont)
    yellow_area = find_size(yellow_cont)
    red_area = find_size(red_cont)

    if (green_area >= yellow_area) and (green_area >= red_area):
        largest = green_area

    elif (yellow_area >= green_area) and (yellow_area >= red_area):
        largest = yellow_area
    else:
        largest = red_area

    if largest == green_area:
        return 0
    elif largest == yellow_area:
        return 1
    elif largest == red_area:
        return 2
