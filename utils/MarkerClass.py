import numpy as np


class Marker:

    def __init__(self, corners, ids):
        self.corners = corners
        self.ids = ids
        self.average_length = 0
        self.centroid = (0, 0)
        self.position = ""
        self.orientation = ""
        self.slope = 0

    def average_length_func(self):
        l_1 = np.sqrt(pow(abs(self.corners[0][0][0][0] - self.corners[0][0][1][0]), 2) + pow(
            abs(self.corners[0][0][0][1] - self.corners[0][0][1][1]), 2))

        l_2 = np.sqrt(pow(abs(self.corners[0][0][1][0] - self.corners[0][0][2][0]), 2) + pow(
            abs(self.corners[0][0][1][1] - self.corners[0][0][2][1]), 2))

        l_3 = np.sqrt(pow(abs(self.corners[0][0][2][0] - self.corners[0][0][3][0]), 2) + pow(
            abs(self.corners[0][0][2][1] - self.corners[0][0][3][1]), 2))

        l_4 = np.sqrt(pow(abs(self.corners[0][0][3][0] - self.corners[0][0][0][0]), 2) + pow(
            abs(self.corners[0][0][3][1] - self.corners[0][0][0][1]), 2))

        average_length = (l_1 + l_2 + l_3 + l_4) * 0.25

        self.average_length = average_length

    def centroid_func(self):
        x_sum = self.corners[0][0][0][0] + self.corners[0][0][1][0] + self.corners[0][0][2][0] + self.corners[0][0][3][
            0]

        y_sum = self.corners[0][0][0][1] + self.corners[0][0][1][1] + self.corners[0][0][2][1] + self.corners[0][0][3][
            1]

        x_centroid = x_sum * 0.25
        y_centroid = y_sum * 0.25

        self.centroid = x_centroid, y_centroid

    def position_func(self, position_offset=0):
        global position_x, position_y

        x, y = (360, 240)

        if x - self.centroid[0] > 0 + position_offset:
            position_x = "Left"

        elif x - self.centroid[0] < 0 + position_offset:
            position_x = "Right"

        else:
            position_x = "Center"

        if y - self.centroid[1] > 0 + position_offset:
            position_y = "Up"

        elif y - self.centroid[1] < 0 + position_offset:
            position_y = "Down"

        else:
            position_y = "Center"

        self.position = position_x + ", " + position_y

        return position_x, position_y

    def orientation_func(self, orientation_offset=0):

        x1 = self.corners[0][0][2][0]

        x2 = self.corners[0][0][3][0]

        y1 = self.corners[0][0][2][1]

        y2 = self.corners[0][0][3][1]

        slope = (y2 - y1) / (x2 - x1)

        self.slope = slope

        if slope > 0 + orientation_offset:
            self.orientation = "Slanted Right"

        elif slope < 0 + orientation_offset:
            self.orientation = "Slanted Left"

        else:
            self.orientation = "Center"
