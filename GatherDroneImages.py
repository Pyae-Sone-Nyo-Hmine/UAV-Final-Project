import djitellopy as tello
import cv2
import numpy as np
import time
from utils.build import *

# Picture Settings
person = 'Max'  # change your name
LED_color = "Red"  # change for which LED your using
counter = 0


# Initialize my_drone
my_drone = tello.Tello()

# Connect to my_drone
my_drone.connect()

# Check battery
print("Battery : " + str(my_drone.get_battery()))

# Turn on video streaming
my_drone.streamon()

time.sleep(10)  # 10 second sleep

while True:
    img = my_drone.get_frame_read().frame
    img = cv2.resize(img, (720, 480))
    cv2.imshow("Tello Stream", img)
    cv2.imwrite("/Users/maxzhang/Downloads/" + person + LED_color + str(counter) + ".png", img)
    cv2.waitKey(100)  # sleeps 250 ms

    counter += 1

    if counter == 200:
        break