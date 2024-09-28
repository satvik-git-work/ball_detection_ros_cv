#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import numpy as np

bridge = CvBridge() #using CvBridge for converting ROS img into CV compatible img and vice-versa

def read_rgb_image(image_name): #Function to read a regular rgb image
    rgb_image = cv2.imread(image_name)
    return rgb_image

def filter_color(rgb_image, lower_bound_color, upper_bound_color): #function that perfroms color filtering to generate a binary image
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask

def getContours(binary_image): #function that retrieves all the contours present in an image frame
    contours, hierarchy = cv2.findContours(binary_image.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_ball_contour(binary_image, rgb_image, contours): #function that draws the contours retrieved on the original rgb image
    
    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area>3000):
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
    return rgb_image    

def get_contour_center(contour): #function to extract the centroid of the countour detected
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def detect_ball_in_a_frame(image_frame): #function that detects the ball using several image processing techniques
    yellowLower =(30, 100, 50)
    yellowUpper = (60, 255, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    contours = getContours(binary_image_mask)
    final_img=draw_ball_contour(binary_image_mask, rgb_image,contours)
    return final_img


def img_callback(msg:Image) : #callback function that receives published image data through subscription
    try: 
        cv_img=bridge.imgmsg_to_cv2(msg,'bgr8')
        processed_image = process_image(cv_img)
        op_img=bridge.cv2_to_imgmsg(processed_image,'bgr8')
        pub.publish(op_img)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def process_image(cv_img): #function that passes the ros img msg for image processing to achieve ball detection
    detected_img=detect_ball_in_a_frame(cv_img)
    return detected_img        


if __name__ == '__main__':
    rospy.init_node("detect_ball_node")#initialization of the detect_ball_node node
    rospy.loginfo("node has begun")

    pub=rospy.Publisher("/output_video",Image,queue_size=10)#Publisher initialization
    sub=rospy.Subscriber("/input_video", Image, img_callback)#Subscriber initialization

    rospy.spin()
