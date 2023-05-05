#!/usr/bin/env python

from tf.transformations import quaternion_from_euler
import cv2
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from darknet_ros_msgs.msg import BoundingBoxes
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
position = []
def callback_function(data):
    client.wait_for_server()
    for element in data.bounding_boxes:
        print(element)
        #resolution 640x480
        xc=element.xmin+((element.xmax-element.xmin)/2)
        yc=element.ymin+((element.ymax-element.ymin)/2)
        dist=depth_image[yc,xc]
        if element.Class == "diningtable":
            print("TROVATO!!!!!")
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "camera_link"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = dist/2
            goal.target_pose.pose.orientation.w = 1
            client.send_goal(goal)
            wait = client.wait_for_result(rospy.Duration(10))
            rospy.signal_shutdown("callback_executed")

            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                return client.get_result()   
             

def get_distance(img):
       global depth_image
       bridge=CvBridge()
       depth_image = bridge.imgmsg_to_cv2(img, desired_encoding="32FC1")
       

def getting_data():
    rospy.Subscriber('/camera1/depth/image_raw',Image,get_distance)
    img_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback_function)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("darknet_subscriber_test")
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    try:
        getting_data()
    except KeyboardInterrupt:
        print("Shutting down")
