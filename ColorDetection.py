import djitellopy as tello
import cv2
import numpy as np

my_tello = tello.Tello()
my_tello.connect()

my_tello.streamon()

while True:
    img = my_tello.get_frame_read().frame
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([40, 50, 50])
    upper = np.array([85, 100, 255])
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(img, img, mask=mask)

    cv2.imshow("Image", img)
    cv2.waitKey(1)
    cv2.imshow("Mask", mask)
    cv2.waitKey(1)
    cv2.imshow("Res", res)
    cv2.waitKey(1)