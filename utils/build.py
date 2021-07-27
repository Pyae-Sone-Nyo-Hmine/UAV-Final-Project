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
def ar_marker_orientation_with_PID(my_ar, kp, ki, kd, prev_x_error, prev_y_error):
    my_ar.centroid_func()

    x_error = my_ar.centroid[0] - 360
    x_speed = kp * x_error + ki * x_error + kd * (x_error - prev_x_error)
    x_speed = int(np.clip(x_speed, -100, 100))

    y_error = my_ar.centroid[1] - 240
    y_speed = kp * y_error + ki * y_error + kd * (y_error - prev_y_error)
    y_speed = -(int(np.clip(y_speed, -100, 100)))

    return x_error, y_error, x_speed, y_speed


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

    s_error = my_ar.slope
    s_speed = kp * s_error + ki * s_error + kd * (s_error - prev_s_error)
    s_speed = int(np.clip(s_speed, -100, 100))

    return s_error, s_speed


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
