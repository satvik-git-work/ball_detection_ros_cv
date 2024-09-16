#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge() #using CvBridge for converting ROS img into CV compatible img and vice-versa


if __name__ == '__main__':
    rospy.init_node('video_publisher_node') #initialization of the video_publisher_node node
    rospy.loginfo("node started")
    img_pub=rospy.Publisher("/input_video",Image,queue_size=10) #Publisher initialization

    video_source=rospy.get_param('video_source','/home/satvik/ros_course/src/ros_essentials_cpp/src/topic03_perception/video/tennis-ball-video.mp4')
    vid_cap=cv2.VideoCapture(video_source)
    rate = rospy.Rate(15) #Frequency: 15 FPS

    while not rospy.is_shutdown():
         ret, frame = vid_cap.read()
         if ret:
              try:
                   inp_img_msg=bridge.cv2_to_imgmsg(frame, "bgr8")
                   img_pub.publish(inp_img_msg)
                   rate.sleep()
              except CvBridgeError as e: 
                   rospy.logerr("CvBridge Error: {0}".format(e))
         else:
              rospy.loginfo("End of video stream or error occurred")
              break
    vid_cap.release()                 

