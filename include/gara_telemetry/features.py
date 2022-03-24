import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge


def get_min_max_point(image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='mono16')
    cv_image = cv2.GaussianBlur(cv_image, (41, 41), 0)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(cv_image)
    return maxLoc, maxVal, minLoc, minVal


def calculate_mean_of_contours(image, mask):
    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
    means = []
    positions = []
    for contour in contours:
        mask_ = np.zeros(image.shape, dtype=np.uint8)
        cv2.fillPoly(mask_, pts=[contour], color=(255, 255, 255))
        means.append(cv2.mean(image, mask_)[0])
        M = cv2.moments(contour)
        print(M['m00'])
        positions.append([int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])])
    return means, positions


def get_max_value(image, max_threshold):
    _, mask = cv2.threshold(image, max_threshold, 255, cv2.THRESH_BINARY)
    means, positions = calculate_mean_of_contours(image, mask)

    mean_max = -1000
    position = [0, 0]
    for mean in means:
        if mean > mean_max:
            mean_max = mean
            position = positions[means.index(mean)]

    return mean_max, position


def get_min_value(image, min_threshold):
    _, mask = cv2.threshold(image, min_threshold, 255, cv2.THRESH_BINARY)
    means, positions = calculate_mean_of_contours(image, cv2.bitwise_not(mask))
    mean_max = 1000
    position = [0, 0]
    for mean in means:
        if mean < mean_max:
            mean_max = mean
            position = positions[means.index(mean)]

    return mean_max, position


def get_max_min_values(image, max_threshold, min_threshold):
    blur = cv2.GaussianBlur(image, (41, 41), 0)

    max_value, position_max = get_max_value(blur, max_threshold)

    min_value, position_min = get_min_value(blur, min_threshold)

    return position_max, max_value, position_min, min_value


def calculate_temperature_by_areas(image, max_threshold=200, min_threshold=40):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
    return get_max_min_values(cv_image, max_threshold, min_threshold)


def get_temperature(value, max_temp, min_temp, max_image_value):
    normalized = value / max_image_value
    return (normalized * (max_temp - min_temp)) + min_temp
