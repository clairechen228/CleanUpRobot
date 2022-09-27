#! /usr/bin/python3

import rospy
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from collections import OrderedDict
from scipy.spatial import distance as dist
import imutils
import numpy as np

# Instantiate CvBridge
bridge = CvBridge()


def detect_shape(c):
    shape = "unidentified"
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)
    # if the shape is a triangle, it will have 3 vertices
    if len(approx) == 3:
        shape = "triangle"
    # if the shape has 4 vertices, it is either a square or
    # a rectangle
    elif len(approx) == 4:
        # compute the bounding box of the contour and use the
        # bounding box to compute the aspect ratio
        (x, y, w, h) = cv2.boundingRect(approx)
        ar = w / float(h)
        # a square will have an aspect ratio that is approximately
        # equal to one, otherwise, the shape is a rectangle
        shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
    # if the shape is a pentagon, it will have 5 vertices
    elif len(approx) == 5:
        shape = "pentagon"
    # otherwise, we assume the shape is a circle
    else:
        shape = "circle"
    # return the name of the shape
    return shape

colors = OrderedDict({
            "blue": (255, 0, 0),
            "green": (0, 255, 0),
            "red": (0, 0, 255),
            "cyan": (255,255,0),
            "yellow": (0, 255, 255),
            "pink": (255, 0, 255),})
# allocate memory for the L*a*b* image, then initialize
# the color names list
lab = np.zeros((len(colors), 1, 3), dtype="uint8")
colorNames = []
# loop over the colors dictionary
for (i, (name, rgb)) in enumerate(colors.items()):
    # update the L*a*b* array and the color names list
    lab[i] = rgb
    colorNames.append(name)
# convert the L*a*b* array from the RGB color space
# to L*a*b*
# lab = cv2.cvtColor(lab, cv2.COLOR_RGB2LAB)

def label_color(image, c):
    # construct a mask for the contour, then compute the
    # average L*a*b* value for the masked region
    mask = np.zeros(image.shape[:2], dtype="uint8")
    img = cv2.drawContours(mask, [c], -1, 255, -1)
    # plt.imshow(img)
    # plt.show()
    mask = cv2.erode(mask, None, iterations=2)
    mean = cv2.mean(image, mask=mask)[:3]
    # initialize the minimum distance found thus far
    minDist = (np.inf, None)
    # loop over the known L*a*b* color values
    for (i, row) in enumerate(lab):
        # compute the distance between the current L*a*b*
        # color value and the mean of the image
        d = dist.euclidean(row[0], mean)
        # if the distance is smaller than the current distance,
        # then update the bookkeeping variable
        if d < minDist[0]:
            minDist = (d, i)
    # return the name of the color with the smallest distance
    return colorNames[minDist[1]]


def detect_main(img):
    # cv2.imwrite('/home/yu/temp_img/step1.jpeg', img)
    h, w, c = img.shape
    resized = imutils.resize(img, width=300)
    ratio = img.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    # gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    # cv2.imwrite('/home/yu/temp_img/step3.jpeg', gray)
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # cv2.imwrite('/home/yu/temp_img/step4.jpeg', blurred)
    # thresh = cv2.threshold(blurred, 33, 255, cv2.THRESH_BINARY)[1]

    blue_thresh = cv2.inRange(resized, (230, 0, 0), (255,50,50))
    cyan_thresh = cv2.inRange(resized, (230, 230, 0), (255,255,50))
    green_thresh = cv2.inRange(resized, (0, 230, 0), (50,255,50))
    red_thresh = cv2.inRange(resized, (0, 0, 230), (50,50, 255))
    yellow_thresh = cv2.inRange(resized, (0, 230, 230), (20, 255, 255))
    pink_thresh = cv2.inRange(resized, (230, 0, 160), (255,50, 255))
    thresh = red_thresh | blue_thresh | yellow_thresh | pink_thresh | cyan_thresh | green_thresh
    # thresh = blue_thresh
    # cv2.imwrite('/home/yu/temp_img/step2.jpeg', thresh)

    # find contours in the thresholded image and initialize the
    # shape detector
    cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE,
        cv2.CHAIN_APPROX_SIMPLE)
    # cnts = imutils.grab_contours(cnts)

    ResString = ""

    for c in cnts:
        M = cv2.moments(c)
        try:
            cX = float((M["m10"] / M["m00"]) * ratio)
            cY = float((M["m01"] / M["m00"]) * ratio)
            cX = cX / w - 0.5
            cY = cY / h - 0.5
            shape = detect_shape(c)
            color = label_color(resized, c)
            area = cv2.contourArea(c) * ratio * ratio / (h*w)
            ResString += shape + "," + color + "," + str(area) + "," + str(cX) + "," + str(cY) + ";"
        except:
            pass

        # draw the outline
        # c = c.astype("float")
        # c *= ratio
        # c = c.astype("int")
        # cv2.drawContours(img, [c], -1, (0, 255, 0), 2)  # draw contour
        # cv2.circle(img, (int(cX), int(cY)), 7, (255, 255, 255), -1)  # draw center
        # cv2.putText(img, shape + "," + color, (int(cX), int(cY)), cv2.FONT_HERSHEY_SIMPLEX,
        #             0.5, (255, 255, 255), 2)
    # show the output image
    # cv2.imwrite('/home/yu/temp_img/step3.jpeg', img)

    return ResString



def img_callback_info(msg, args):
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        rec_str = detect_main(cv2_img)
        # cv2.imwrite('/home/yu/temp_img/camera_image.jpeg', cv2_img)

    pub = args
    # if you want to see the msg, log it here:
    # rospy.loginfo(rec_str)

    # publish the msg
    pub.publish(rec_str)
    

def main():
    # init node:
    rospy.init_node("camera_listener")
    # pub cv results:
    pub = rospy.Publisher("rec_result", String, queue_size=10)
    # sub to image input:
    image_topic = "/camera/rgb/image_raw"
    rospy.Subscriber(image_topic, Image, img_callback_info, (pub))

    rospy.spin()


if __name__ == '__main__':
    main()
