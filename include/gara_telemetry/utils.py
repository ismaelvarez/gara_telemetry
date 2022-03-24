import cv2
from cv_bridge import CvBridge


def draw_min_max_temp(image, max_position, min_position):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

    cv2.circle(cv_image, tuple(max_position), 5, (0, 0, 255), 2)
    cv2.circle(cv_image, tuple(min_position), 5, (255, 0, 0), 2)

    return bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")


def draw(image, mask):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    # Adaptive threshold
    # cv_image = cv2.adaptiveThreshold(cv_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 99, 3)
    im_thresh_gray = cv2.bitwise_and(cv_image, mask)
    blur = cv2.GaussianBlur(cv_image, (5, 5), 0)
    ret3, cv_image = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # cv2.circle(cv_image, max_position, 5, (0, 0, 255), 2)
    # cv2.circle(cv_image, min_position, 5, (255, 0, 0), 2)

    return bridge.cv2_to_imgmsg(im_thresh_gray, encoding="bgr8")
