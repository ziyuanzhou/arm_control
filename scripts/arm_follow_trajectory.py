#!/usr/bin/env python

import rospy
import actionlib

#import all message types we need from ROS
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from math import pi
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryActionFeedback,
    FollowJointTrajectoryActionResult,
    GripperCommandAction,
    GripperCommand,
)

from actionlib_msgs.msg import (
    GoalStatus,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

# uncomment the following line if you are using real hardware
import Adafruit_GPIO.FT232H as FT232H


#own code starting here:
# define a class that listens to MoveIt and drives the robotic arm through SPI
# It uses ROS actionlib, if you are not familiar with ROS actionlib, please
# go to http://wiki.ros.org/actionlib/Tutorials and learn the tutorials that
# walk you through how to write action server and client.
# Our class ArmFollowTrajectory is an action server, and the moveit node is a client. 
# Our class listens to moveit and provides actions when being called by
# MoveIt.
class ArmFollowTrajectory(object):
    # A python class can have as many functions as you want. But the 
    # necessary one is __init__(self), which is automatically called
    # when an object of this class in being created, in the main function
    def __init__(self):
        ## In the init function, we create variables, create action servers
        ## create publishers and initialize SPI communication
        
        ### Initialize and create SPI ###
        # Uncomment the following graph if you are using real hardware
        # Temporarily disable FTDI serial drivers.
        FT232H.use_FT232H()
        # Find the first FT232H device.
        # the device has to be attached to /dev/ttyUSB0 in Linux
        ft232h = FT232H.FT232H()
        # Create a SPI interface from the FT232H using pin 8 (C0) as chip select.
        # Use a clock speed of 3mhz, SPI mode 0, and most significant bit first.
        spi = FT232H.SPI(ft232h, cs=8, max_speed_hz=3000000, mode=0, bitorder=FT232H.MSBFIRST)
        self.spi = spi
        
        
        ### Define and initialize ROS action servers and ROS topic publishers ###
        # define arm action server, of type FollowJointTrajectoryAction
        # using a callback function of execute_cb_arm
        # communicating in the namespace of /arm_controller/follow_joint_trajectory
        # notice this namespace cannot be changed, because its being determined 
        # by MoveIt, as can be seen here: http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/controller_configuration/controller_configuration_tutorial.html
        self._as_arm = actionlib.SimpleActionServer('/arm_controller/follow_joint_trajectory',
                                                    FollowJointTrajectoryAction,
                                                    execute_cb=self.execute_cb_arm,
                                                    auto_start = False)
        self._as_arm.start()
        
        # define hand action server, of type GripperCommandAction
        # using a callback function of self.execute_cb_hand
        # communicating in the namespace of /hand_controller/gripper_action
        # notice this namespace cannot be changed, because its being determined
        # by MoveIt, as can be seen here: http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/controller_configuration/controller_configuration_tutorial.html
        self._as_hand = actionlib.SimpleActionServer('/hand_controller/gripper_action',
                                                      GripperCommandAction,
                                                      execute_cb=self.execute_cb_hand,
                                                      auto_start = False)
        self._as_hand.start()
        
        ### declare some variables that are stored in this class ###
        # joint_names defined in the URDF file, in the order of from
        # joint 1 to joint 6
        self.joint_names = ['link_1_a_to_base', 'link_2_to_link_1', 'link_3_to_link_2',
                            'link_4_to_link_3', 'link_4_c_to_b', 'finger_1_to_link_5']
        
        # during initialization, all jointStates are set to 0, the defalt position
        # the corresponding PWMs are all 1500us.
        self.jointStates = [0,0,0,0,0,0] 
        self.jointPWMs = [1500, 1500, 1500, 1500, 1500, 1500]
        self.k = [1400.0/pi, -1200.0/pi, 1200.0/pi, 1200.0/pi, 1400.0/pi, 1200.0/pi]
        self.b = [1500, 1500, 1500, 1500, 1500, 1500]
        
        # we then send the initialization default PWMs to SPI to drive 
        # the robotic arm. Uncomment this if you are using real hardware.
        self.spi_send_PWM() 
        
        
        ### define joint state publisher ###
        # since our robot servos move to the exact joint angles as what
        # they told to, so our joint state publisher just echo the goal
        # that is being sent from MoveIt
        self.pub_PWM = rospy.Publisher('/summer_robotic_arm/jointPWMs', 
                                        JointState, 
                                        queue_size=10) ## we use ros's built-in joint state message type for our jointPWMs
        
        self.joint_PWM_msg = JointState()
        self.joint_PWM_msg.name = self.joint_names
        self.joint_PWM_msg.position = self.jointPWMs
        self.pub_PWM.publish(self.joint_PWM_msg)
        '''
        self.pub_state = rospy.Publisher('/summer_robotic_arm/jointStates', 
                                        JointState, 
                                        queue_size=10) ## we use ros's built-in joint state message type for our jointPWMs
        '''
        # notice that the name space /joint_states namespace also controls
        # the rViz visualization, which makes our simulation the same
        # as the real robot
        self.pub_state = rospy.Publisher('/joint_states', 
                                        JointState, 
                                        queue_size=10) ## we use ros's built-in joint state message type for our jointPWMs
        
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_msg.position = self.jointStates
        self.pub_state.publish(self.joint_state_msg)
    
    # the arm callback function, which means everytime MoveIt plans and
    # executes a trajectory, a message goal will be sent to the action
    # server, and then this function will be triggered.
    # In the callback function of an action server, we need to
    # send feedback information to client in real time. And when
    # this action is completed, we need to send result information to the
    # client.
    def execute_cb_arm(self, goal):
        # create feedback msg
        self.arm_feedback = FollowJointTrajectoryFeedback()
        self.arm_feedback.joint_names = goal.trajectory.joint_names
        ## notice that the order in goal.trajectory.joint_names is different
        ## from the default order in self.joint_names!!!!
        #~ print goal.trajectory.joint_names
        print "arm"
        #~ print len(goal.trajectory.points)
        print goal.trajectory.points
        
        # we execute the trajectory at 5Hz, which means we execute 5
        # points per second
        action_rate = rospy.Rate(20)
        
        ## for each point in the planned trajectory, we excute it through
        ## SPI and send feedback
        for trajectory_point in goal.trajectory.points:
            # the actual jointstate is same as the required jointstate
            self.arm_feedback.actual = trajectory_point
            # we send the feedback infomation back to the client
            self._as_arm.publish_feedback(self.arm_feedback)
            
            # send the jointstates through SPI in the correct order !!!!
            for i in range(len(self.jointStates)-1):
                self.jointStates[i] = trajectory_point.positions[goal.trajectory.joint_names.index(self.joint_names[i])]
            print self.jointStates
           
            ########################## IMPORTANT #######################
            ### add your own linear mapping between self.jointStates and self.jointPWMs
            ### self.jointPWWs = f(self.jointStates)
            self.jointPWMs = np.array(self.k) * np.array(self.jointStates) + np.array(self.b)
            print self.jointPWMs
            ############################################################
            
            # we publish the updated joint states, with the correct time stamps
            self.joint_state_msg.header.stamp = rospy.Time.now()
            self.joint_state_msg.position = self.jointStates
            self.pub_state.publish(self.joint_state_msg)
            self.joint_PWM_msg.position = self.jointPWMs
            self.pub_PWM.publish(self.joint_PWM_msg)
            
            # send PWMs to SPI
            # uncomment this if you are using real hardware and sending
            # data to Arduino through SPI
            self.spi_send_PWM()
            
            action_rate.sleep()
        
        # After executing every trajectory point sent by the client
        # we notify the client that the action is being completed.    
        self.arm_result = FollowJointTrajectoryResult()
        self.arm_result.error_code = 0 # error code 0 means successful
        self._as_arm.set_succeeded(self.arm_result)
    
    # the hand callback function, which is not being used right now.
    # we will finish it later
    def execute_cb_hand(self, command):
        print "hand"
        ### Add SPI communication layer here
        self.jointStates = self.jointStates[:-1].append(command.position)
        ### add your own linear mapping between self.jointStates and self.jointPWMs
        ### self.jointPWWs = f(self.jointStates)
        #~ self.spi_send_PWM()
        
    # help function that sends joint PMWs to Arduino through SPI to
    # drive the hardware
    def spi_send_PWM(self):
        ## Notice we cannot keep writing data through SPI, because that will make
        ## the robotic arm get crazy. So we only write it 3 times, which ensures
        ## the robotic arm can get the signal but not go crazy.
        # write joint PWMs to SPI for arduino to read and drive servos
        # sent two NULL bytes at the beginning of the communication to
        # indicate the beginning of the data array. Arduino uses these
        # two NULL bytes as a flag
        for i in range(3):
            print "SPI sending"
            self.spi.write([0x00])
            self.spi.write([0x00])
            # we break each number into two bytes and send them separately
            # because in SPI, data is sent byte by byte
            for i in range(len(self.jointPWMs)):
                pwm = int(self.jointPWMs[i])  # take each number in the array
                msb = (pwm >> 8) & 0xff # grab the most significant byte
                lsb = pwm & 0xff # grab the least significant byte
                self.spi.write([msb]) # write MSB
                self.spi.write([lsb]) # write LSB
        
        
if __name__ == '__main__':
    rospy.init_node('arm_control_server', anonymous=False)
    server = ArmFollowTrajectory()
    #~ rospy.spin()
    
    # we keep broadcasting joint state messages
    while not rospy.is_shutdown():
        #~ print "running"
        server.joint_state_msg.header.stamp = rospy.Time.now()
        server.pub_PWM.publish(server.joint_PWM_msg)
        server.pub_state.publish(server.joint_state_msg)
