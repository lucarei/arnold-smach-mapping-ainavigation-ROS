#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension


if __name__ == '__main__':
    rospy.init_node('move_for_mapping')
    rospy.loginfo("Starting movements...")
    # Publisher
    pub_joint_values = rospy.Publisher('/planner_pipeline/target_joint_path', Float32MultiArray, queue_size = 10)
    rospy.sleep(1)
    
    joints = Float32MultiArray()
    joints.data = [ 1.57, -0.5, -1.6, 0.4, -1.57, 0.0,
                    0.77, -0.5, -1.6, 0.4, -1.57, 0.0,
                    2.37, -0.5, -1.6, 0.4, -1.57, 0.0,
                    1.57, -0.5, -1.6, 0.0, -1.57, 0.0,
                    0.77, -0.5, -1.6, 0.0, -1.57, 0.0,
                    2.37, -0.5, -1.6, 0.0, -1.57, 0.0,
                    1.57, -0.5, -1.6, -0.4, -1.57, 0.0,
                    0.77, -0.5, -1.6, -0.4, -1.57, 0.0,
                    2.37, -0.5, -1.6, -0.4, -1.57, 0.0,
                    1.57, -0.5, -1.6, -0.65, -1.57, 0.0,
                    0.77, -0.5, -1.6, -0.65, -1.57, 0.0,
                    2.37, -0.5, -1.6, -0.65, -1.57, 0.0,
                    1.57, -1.0, -2.0, -0.9, -1.57, 0.0,
                    0.77, -1.0, -2.0, -0.9, -1.57, 0.0,
                    2.37, -1.0, -2.0, -0.9, -1.57, 0.0,
                    1.57, -0.5, -1.6, 0.4, -1.57, 0.0]

    for i in range(len(joints.data)/6):
        dimensions = MultiArrayDimension()
        dimensions.label = ""
        dimensions.size = 6
        dimensions.stride = 1
        joints.layout.dim.append(dimensions)

    joints.layout.data_offset = 0
    pub_joint_values.publish(joints)
