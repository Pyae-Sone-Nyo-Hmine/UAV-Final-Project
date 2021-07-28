import djitellopy as tello
import cv2
import numpy as np

my_tello = tello.Tello()
my_tello.connect()

my_tello.streamon()


def find_size(contours):
    if len(contours) == 0:
        return 0

    con_x = list([i[0] for i in contours])[0][0]
    con_y = list([i[1] for i in contours])[0][0]
    print(con_x)
    print(con_y)

    max_x = con_x[0]
    min_x = con_x[0]
    print(max_x )
    print(min_x)

    max_y = con_y[0]
    min_y = con_y[0]
    print(max_y)
    print(min_y)

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


def find_largest(green_cont,yellow_cont,red_cont):
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


while True:
    kernel = np.ones((7, 7), np.uint8)

    img = my_tello.get_frame_read().frame
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_green = np.array([40, 35, 90])# adjust as needed
    upper_green = np.array([85, 255, 255])# adjust as needed
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    res_green = cv2.bitwise_and(img, img, mask=mask_green)
    opening_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    contours_green, _ = cv2.findContours(opening_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)




    lower_yellow = np.array([20, 35, 90])# adjust as needed
    upper_yellow = np.array([26, 255, 255])# adjust as needed
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    res_yellow = cv2.bitwise_and(img, img, mask=mask_yellow)
    opening_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
    contours_yellow, _ = cv2.findContours(opening_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)



    lower_red1 = np.array([0, 25, 90])# adjust as needed
    upper_red1 = np.array([8, 255, 255])# adjust as needed
    lower_red2 = np.array([173, 25, 90])# adjust as needed
    upper_red2 = np.array([180, 255, 255])# adjust as needed
    mask1_red = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2_red = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask1_red, mask2_red)
    res_red = cv2.bitwise_and(img, img, mask=mask_red)
    opening_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    contours_red, _ = cv2.findContours(opening_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if find_largest(contours_green, contours_yellow, contours_red) == 0:
        print("green is on")
    elif find_largest(contours_green, contours_yellow, contours_red) == 1:
        print("yellow is on")
    elif find_largest(contours_green, contours_yellow, contours_red) == 2:
        print("red is on")

    cv2.imshow("Image", img)
    cv2.waitKey(1)
    cv2.imshow("Mask ", mask_green)
    cv2.waitKey(1)
    cv2.imshow("Res ", res_green)
    cv2.waitKey(1)