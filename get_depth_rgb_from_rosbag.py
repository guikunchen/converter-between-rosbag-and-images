import roslib
import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
rgb_path = '/media/c/F/dataset/re-edit-bag/rgb/'# absolute path of extracted rgb images
depth_path = '/media/c/F/dataset/re-edit-bag/depth/'# absolute path of extracted depth images
bridge = CvBridge()
with rosbag.Bag('/media/c/F/dataset/re-edit-bag/demo.bag', 'r') as bag:
    for topic,msg,t in bag.read_messages():
        if topic == "/camera/aligned_depth_to_color/image_raw": 
            cv_image = bridge.imgmsg_to_cv2(msg, '16UC1')
            timestr = "%.8f" %  msg.header.stamp.to_sec()
            image_name = timestr + '.png'# an extension is necessary
            cv2.imwrite(depth_path + image_name, cv_image)
            print(depth_path + image_name)
        if topic == "/camera/color/image_raw": 
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            timestr = "%.8f" %  msg.header.stamp.to_sec()
            image_name = timestr + '.jpg'# an extension is necessary
            cv2.imwrite(rgb_path + image_name, cv_image)
