from djitellopy import tello
from utils.build import *

# Initialize my_drone
my_drone = tello.Tello()

# Connect to my_drone
my_drone.connect()

# Check battery
print("Battery : " + str(my_drone.get_battery()))

# Turn on video streaming
my_drone.streamon()

# Turn on functions
control_drone = True
aruco_tracking = True
PID = True
follow_AR = True

# constants for pid
prev_x_error = prev_y_error = prev_d_error = 0

# MAIN LOOP
while True:
    img = my_drone.get_frame_read().frame
    img = cv2.resize(img, (720, 480))
    corners, ids = ar_detector(img)
    my_ar = Marker(corners, ids)
    cv2.imshow("Tello Stream", img)
    cv2.waitKey(1)

    if control_drone:

        if aruco_tracking:

            if PID:

                try:
                    values = motion_func(my_drone=my_drone)
                    prev_x_error, prev_y_error, x_speed, y_speed = ar_marker_orientation_with_PID(my_ar, 0.3, 0, 0.1,
                                                                                                  prev_x_error,
                                                                                                  prev_y_error)
                    if follow_AR:
                        prev_d_error, fb_speed = follow_ar_with_PID(my_ar, 0.25, 0, 0.5, prev_d_error)
                        my_drone.send_rc_control(values[0], fb_speed + values[1], y_speed + values[2],
                                                 x_speed + values[3])
                    else:
                        my_drone.send_rc_control(values[0], values[1], y_speed + values[2], x_speed + values[3])

                except:
                    values = motion_func(my_drone=my_drone)
                    my_drone.send_rc_control(values[0], values[1], values[2], values[3])

            else:
                values_1 = motion_func(my_drone=my_drone)
                values_2 = ar_marker_orientation(my_ar)
                values = [values_1[0] + values_2[0], values_1[1] + values_2[1], values_1[2] + values_2[2],
                          values_1[3] + values_2[3], ]
                my_drone.send_rc_control(values[0], values[1], values[2], values[3])

        else:
            values = motion_func(my_drone=my_drone)
            my_drone.send_rc_control(values[0], values[1], values[2], values[3])
