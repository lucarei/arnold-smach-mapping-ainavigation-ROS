#!/usr/bin/env python

import rospy
import smach
import sys
import smach_ros
import roslaunch
from geometry_msgs.msg import Twist
import os
from std_msgs.msg import Int8
import time
from std_msgs.msg import String



class Initializing(smach.State):
     def __init__(self):
         smach.State.__init__(self, outcomes=['to_check','end'])
        
     def execute(self, userdata):
         rospy.loginfo('Initializing system')
         i=int(input('Per startare la macchina a stati (1): '))
         if i==1:
             uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
             roslaunch.configure_logging(uuid)
             launch_file = ['autonomous_navigation_pkg', 'base_nodes.launch']
             roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)
             parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
             parent.start()
             uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
             roslaunch.configure_logging(uuid)
             launch_file = ['toni_env_exploration', 'save_maps.launch']
             roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)
             parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
             parent.start()
             return 'to_check'
         else:
     	    return 'end'

class Check(smach.State):
     def __init__(self):
         smach.State.__init__(self, outcomes=['go_to_kill','end'])

     def execute(self, userdata):
         rospy.loginfo('Checking if exploration has ended...')
         a=rospy.wait_for_message("/exploration_status", Int8)
         print("---->",a)
         b=int(a.data)
         if b==1:
             return 'go_to_kill'
         else:
             return 'end'
        

class Kill_and_Intersection(smach.State):
     def __init__(self):
         smach.State.__init__(self,outcomes=['arm_start'])

     def execute(self, userdata):
         rospy.loginfo('Intersection phase and killing assembler nodes...')
         
         time.sleep(120)
         rospy.loginfo('2 minutes passed. Starting to kill...')
         os.system('rosnode kill /point_cloud_environment_assembler')
         os.system('rosnode kill /point_cloud_uvc_assembler')
         os.system('rosnode kill /point_cloud_lamp_assembler')
         os.system('rosnode kill /pointcloud_saver')
         os.system('rosnode kill /uvc_dosage_saver')
         os.system('rosnode kill /lamp_dosage_saver')

         uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
         roslaunch.configure_logging(uuid)
         time.sleep(10)
         launch_file = ['toni_uvc_sanification', 'intersection.launch']
         roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)
         parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
         parent.start()
         return 'arm_start'

class Initial_mapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['saving_mapping','end'])

    def execute(self, userdata):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = ['autonomous_navigation_pkg', 'arm_mapping.launch']
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        time.sleep(180)
        riberino=1
        if riberino==1:
            return 'saving_mapping'
        else:
            return 'end'

class Save(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cleaning_phase','end'])

    def execute(self, userdata):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = ['autonomous_navigation_pkg', 'saving_pointcloud.launch']
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        time.sleep(7) #con 10 ne salva due
        os.system('rosnode kill /pointcloud_saver')
        ribera = True
        if ribera:
            return 'cleaning_phase'
        else:
            return 'end'
        

class Clean(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'])

    def execute(self, userdata):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = ['autonomous_navigation_pkg', 'start_clean.launch']
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        input("Fine operazione")
        return 'end'  


def main():
    
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', Initializing(), 
                                transitions={'to_check':'CHECK', 'end':'finished'})
        smach.StateMachine.add('CHECK', Check(), 
                                transitions={'go_to_kill':'KILL_AND_INTER','end':'finished'})
        smach.StateMachine.add('KILL_AND_INTER', Kill_and_Intersection(), 
                               transitions={'arm_start':'INITIAL_MAPPING'})
       
        # Adding states for the aubo arm
        smach.StateMachine.add("INITIAL_MAPPING", Initial_mapping(),
                                 transitions={'saving_mapping':'SAVING_STATE','end':'finished'})
        smach.StateMachine.add("SAVING_STATE",Save(),
                                 transitions={'cleaning_phase':'CLEAN','end':'finished'})
        smach.StateMachine.add("CLEAN", Clean(), transitions={'end':'finished'})
    
    #Debug per smach   
    # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()
    
    # rospy.spin()
    # sis.stop()
    # Execute SMACH plan
    outcome = sm.execute()
    
if __name__ == '__main__':
    main()
