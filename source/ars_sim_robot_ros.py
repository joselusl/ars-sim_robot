#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Accel
from geometry_msgs.msg import AccelStamped

import tf2_ros




#
from ars_sim_robot import *


#
import ars_lib_helpers





class ArsSimRobotRos:

  #######


  # Robot frame
  robot_frame = 'robot_base_link'

  # World frame
  world_frame = 'world'

  # Sim step 
  # time step
  sim_step_time = 0.01
  # Timer
  sim_step_timer = None

  # Pub timer
  # time step
  pub_step_time = 0.02
  # Timer
  pub_step_timer = None

  # Robot command
  flag_sub_robot_vel_cmd_unstamped = False
  robot_vel_cmd_unstamped_sub = None
  robot_vel_cmd_stamped_sub = None

  # Robot pose publisher
  robot_pose_pub = None

  # Robot velocity publisher
  robot_vel_world_pub = None
  robot_vel_robot_pub = None

  # Robot acceleration publisher
  robot_acc_world_pub = None
  robot_acc_robot_pub = None

  # Robot collision subscriber
  robot_collision_sub = None

  # tf2 broadcaster
  tf2_broadcaster = None


  # Robot simulator
  robot = ArsSimRobot()
  


  #########

  def __init__(self):

    return


  def init(self, node_name='ars_sim_robot_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_sim_robot')
    

    #### READING PARAMETERS ###
    

    ###

    #
    self.robot.setTimeStamp(rospy.Time.now())


    # Init state
    self.robot.robot_posi = [0.0, 0.0, 1.0]
    self.robot.robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    self.robot.robot_velo_lin_robot = [0.0, 0.0, 0.0]
    self.robot.robot_velo_ang_robot = [0.0, 0.0, 0.0]

    
    # End
    return


  def open(self):


    # Subcribers
    # Robot cmd subscriber
    if(self.flag_sub_robot_vel_cmd_unstamped):
      self.robot_vel_cmd_unstamped_sub = rospy.Subscriber('robot_cmd', Twist, self.robotVelCmdUnstampedCallback)
    # Robot cmd stamped subscriber
    self.robot_vel_cmd_stamped_sub = rospy.Subscriber('robot_cmd_stamped', TwistStamped, self.robotVelCmdStampedCallback)
    # Robot collision
    self.robot_collision_sub = rospy.Subscriber('robot_collision', Bool, self.robotCollisionCallback)


    # Publishers
    #
    self.robot_pose_pub = rospy.Publisher('robot_pose', PoseStamped, queue_size=1)
    #
    self.robot_vel_world_pub = rospy.Publisher('robot_velocity_world', TwistStamped, queue_size=1)
    #
    self.robot_vel_robot_pub = rospy.Publisher('robot_velocity_robot', TwistStamped, queue_size=1)
    #
    self.robot_acc_world_pub = rospy.Publisher('robot_acceleration_world', AccelStamped, queue_size=1)
    #
    self.robot_acc_robot_pub = rospy.Publisher('robot_acceleration_robot', AccelStamped, queue_size=1)


    # Tf2 broadcasters
    self.tf2_broadcaster = tf2_ros.TransformBroadcaster()


    # Timers
    #
    self.sim_step_timer = rospy.Timer(rospy.Duration(self.sim_step_time), self.simStepTimerCallback)
    #
    self.pub_step_timer = rospy.Timer(rospy.Duration(self.pub_step_time), self.pubStepTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def simStepTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    # Robot
    self.robot.simRobot(time_stamp_current)
    
    # End
    return

  def pubStepTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    # Robot
    self.robot.simRobot(time_stamp_current)

    # Publish
    # Pose
    self.robotPosePub()
    # Velocity
    self.robotVelPub()
    # Acceleration
    self.robotAccPub()

    # End
    return

    
  def robotVelCmdUnstampedCallback(self, robot_vel_cmd_msg):

    robot_velo_lin_cmd = np.zeros((3,), dtype=float)
    robot_velo_lin_cmd[0] = robot_vel_cmd_msg.linear.x
    robot_velo_lin_cmd[1] = robot_vel_cmd_msg.linear.y
    robot_velo_lin_cmd[2] = robot_vel_cmd_msg.linear.z

    robot_velo_ang_cmd = np.zeros((3,), dtype=float)
    robot_velo_ang_cmd[0] = robot_vel_cmd_msg.angular.x
    robot_velo_ang_cmd[1] = robot_vel_cmd_msg.angular.y
    robot_velo_ang_cmd[2] = robot_vel_cmd_msg.angular.z

    self.robot.setRobotVelCmd(robot_velo_lin_cmd, robot_velo_ang_cmd)

    return


  def robotVelCmdStampedCallback(self, robot_vel_cmd_stamped_msg):

    robot_velo_lin_cmd = np.zeros((3,), dtype=float)
    robot_velo_lin_cmd[0] = robot_vel_cmd_stamped_msg.twist.linear.x
    robot_velo_lin_cmd[1] = robot_vel_cmd_stamped_msg.twist.linear.y
    robot_velo_lin_cmd[2] = robot_vel_cmd_stamped_msg.twist.linear.z

    robot_velo_ang_cmd = np.zeros((3,), dtype=float)
    robot_velo_ang_cmd[0] = robot_vel_cmd_stamped_msg.twist.angular.x
    robot_velo_ang_cmd[1] = robot_vel_cmd_stamped_msg.twist.angular.y
    robot_velo_ang_cmd[2] = robot_vel_cmd_stamped_msg.twist.angular.z

    self.robot.setRobotVelCmd(robot_velo_lin_cmd, robot_velo_ang_cmd)

    return


  def robotCollisionCallback(self, robot_collision_msg):

    self.robot.flag_robot_collision = robot_collision_msg.data

    return


  def robotPosePub(self):

    #
    time_stamp_current = self.robot.getTimeStamp()

    #
    robot_posi = self.robot.getRobotPosi()
    robot_atti_quat = self.robot.getRobotAttiQuat()


    #
    robot_pose_msg = PoseStamped()

    robot_pose_msg.header = Header()
    robot_pose_msg.header.stamp = time_stamp_current
    robot_pose_msg.header.frame_id = self.world_frame

    robot_pose_msg.pose.position.x = robot_posi[0]
    robot_pose_msg.pose.position.y = robot_posi[1]
    robot_pose_msg.pose.position.z = robot_posi[2]

    robot_pose_msg.pose.orientation.w = robot_atti_quat[0]
    robot_pose_msg.pose.orientation.x = robot_atti_quat[1]
    robot_pose_msg.pose.orientation.y = robot_atti_quat[2]
    robot_pose_msg.pose.orientation.z = robot_atti_quat[3]

    # Tf2
    tf2__msg = geometry_msgs.msg.TransformStamped()

    tf2__msg.header.stamp = time_stamp_current
    tf2__msg.header.frame_id = self.world_frame
    tf2__msg.child_frame_id = self.robot_frame

    tf2__msg.transform.translation.x = robot_posi[0]
    tf2__msg.transform.translation.y = robot_posi[1]
    tf2__msg.transform.translation.z = robot_posi[2]

    tf2__msg.transform.rotation.w = robot_atti_quat[0]
    tf2__msg.transform.rotation.x = robot_atti_quat[1]
    tf2__msg.transform.rotation.y = robot_atti_quat[2]
    tf2__msg.transform.rotation.z = robot_atti_quat[3]


    self.robot_pose_pub.publish(robot_pose_msg)

    self.tf2_broadcaster.sendTransform(tf2__msg)


    # End
    return


  def robotVelPub(self):

    #
    time_stamp_current = self.robot.getTimeStamp()

    robot_velo_lin_robot = self.robot.getRobotVeloLinRobot()
    robot_velo_ang_robot = self.robot.getRobotVeloAngRobot()

    robot_velo_lin_world = self.robot.getRobotVeloLinWorld()
    robot_velo_ang_world = self.robot.getRobotVeloAngWorld()


    #
    robot_velo_robot_msg = TwistStamped()

    robot_velo_robot_msg.header.stamp = time_stamp_current
    robot_velo_robot_msg.header.frame_id = self.robot_frame

    robot_velo_robot_msg.twist.linear.x = robot_velo_lin_robot[0]
    robot_velo_robot_msg.twist.linear.y = robot_velo_lin_robot[1]
    robot_velo_robot_msg.twist.linear.z = robot_velo_lin_robot[2]

    robot_velo_robot_msg.twist.angular.x = robot_velo_ang_robot[0]
    robot_velo_robot_msg.twist.angular.y = robot_velo_ang_robot[1]
    robot_velo_robot_msg.twist.angular.z = robot_velo_ang_robot[2]

    
    #
    robot_velo_world_msg = TwistStamped()

    robot_velo_world_msg.header.stamp = time_stamp_current
    robot_velo_world_msg.header.frame_id = self.world_frame

    robot_velo_world_msg.twist.linear.x = robot_velo_lin_world[0]
    robot_velo_world_msg.twist.linear.y = robot_velo_lin_world[1]
    robot_velo_world_msg.twist.linear.z = robot_velo_lin_world[2]

    robot_velo_world_msg.twist.angular.x = robot_velo_ang_world[0]
    robot_velo_world_msg.twist.angular.y = robot_velo_ang_world[1]
    robot_velo_world_msg.twist.angular.z = robot_velo_ang_world[2]



    self.robot_vel_robot_pub.publish(robot_velo_robot_msg)

    self.robot_vel_world_pub.publish(robot_velo_world_msg)

    #
    return


  def robotAccPub(self):

    #
    time_stamp_current = self.robot.getTimeStamp()

    robot_acce_lin_robot = self.robot.getRobotAcceLinRobot()
    robot_acce_ang_robot = self.robot.getRobotAcceAngRobot()

    robot_acce_lin_world = self.robot.getRobotAcceLinWorld()
    robot_acce_ang_world = self.robot.getRobotAcceAngWorld()


    #
    robot_acce_robot_msg = AccelStamped()

    robot_acce_robot_msg.header.stamp = time_stamp_current
    robot_acce_robot_msg.header.frame_id = self.robot_frame

    robot_acce_robot_msg.accel.linear.x = robot_acce_lin_robot[0]
    robot_acce_robot_msg.accel.linear.y = robot_acce_lin_robot[1]
    robot_acce_robot_msg.accel.linear.z = robot_acce_lin_robot[2]

    robot_acce_robot_msg.accel.angular.x = robot_acce_ang_robot[0]
    robot_acce_robot_msg.accel.angular.y = robot_acce_ang_robot[1]
    robot_acce_robot_msg.accel.angular.z = robot_acce_ang_robot[2]

    
    #
    robot_acce_world_msg = AccelStamped()

    robot_acce_world_msg.header.stamp = time_stamp_current
    robot_acce_world_msg.header.frame_id = self.world_frame

    robot_acce_world_msg.accel.linear.x = robot_acce_lin_world[0]
    robot_acce_world_msg.accel.linear.y = robot_acce_lin_world[1]
    robot_acce_world_msg.accel.linear.z = robot_acce_lin_world[2]

    robot_acce_world_msg.accel.angular.x = robot_acce_ang_world[0]
    robot_acce_world_msg.accel.angular.y = robot_acce_ang_world[1]
    robot_acce_world_msg.accel.angular.z = robot_acce_ang_world[2]



    self.robot_acc_robot_pub.publish(robot_acce_robot_msg)

    self.robot_acc_world_pub.publish(robot_acce_world_msg)

    #
    return
