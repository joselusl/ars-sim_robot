#!/usr/bin/env python

import numpy as np
from numpy import *

import rospy
import rospkg

import std_msgs.msg
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Header

from ars_robot_msgs.msg import RobotStatus



class ArsSimRobotStatusRos:

  # Robot status
  # UNKNOWN = 0
  # LANDED = 1
  # TAKING_OFF = 2
  # LANDING = 3
  # FLYING = 4
  robot_status = None
  robot_status_types = {'UNKNOWN': 0, 'LANDED': 1, 'TAKING_OFF': 2, 'LANDING': 3, 'FLYING': 4}

  # Timers
  # secs
  taking_off_duration = 3.0
  taking_off_timer = None
  # secs
  landing_duration = 3.0
  landing_timer = None

  #
  robot_takeoff_sub = None
  robot_land_sub = None

  #
  robot_status_pub = None
  robot_cmd_control_enabled_pub = None
  robot_motion_enabled_pub = None

  #
  robot_status_timer = None

  #
  robot_init_status_flying = None



  #########

  def __init__(self):

    self.robot_status = self.robot_status_types['UNKNOWN']

    return


  def init(self, node_name='ars_sim_robot_status_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_sim_robot')
    

    #### READING PARAMETERS ###
    self.robot_init_status_flying = rospy.get_param('~robot_init_status_flying', False)


    # End
    return


  def open(self):

    # Publishers
    #
    self.robot_status_pub = rospy.Publisher("robot_status", RobotStatus, latch=True)
    # Robot cmd control enabled
    self.robot_cmd_control_enabled_pub = rospy.Publisher('robot_cmd_control_enabled', Bool, latch=True)
    # Robot motion enabled
    self.robot_motion_enabled_pub = rospy.Publisher('robot_motion_enabled', Bool, latch=True)


    # Subscribers
    #
    self.robot_takeoff_sub = rospy.Subscriber("takeoff", Empty, self.takeoffCallback, queue_size=1)
    #
    self.robot_land_sub = rospy.Subscriber("land", Empty, self.landCallback, queue_size=1)


    # Timers
    self.robot_status_timer = rospy.Timer(rospy.Duration(0.1), self.robotStatusTimerCallback)



    # Init
    if(self.robot_init_status_flying is True):
      print('Flying')
      self.setRobotFlying()
    else:
      print('Landed')
      self.setRobotLanded()


    # End
    return


  def run(self):

    rospy.spin()

    return


  def setRobotLanded(self):

    # Disable controls (just in case)
    robot_cmd_control_enabled = Bool()
    robot_cmd_control_enabled.data = False
    self.robot_cmd_control_enabled_pub.publish(robot_cmd_control_enabled)
    # Disable Robot motion (just in case)
    robot_motion_enabled = Bool()
    robot_motion_enabled.data = False
    self.robot_motion_enabled_pub.publish(robot_motion_enabled)

    # Change status
    self.setRobotStatus(self.robot_status_types['LANDED'])

    return


  def setRobotFlying(self):

    # Enable controls
    robot_cmd_control_enabled = Bool()
    robot_cmd_control_enabled.data = True
    self.robot_cmd_control_enabled_pub.publish(robot_cmd_control_enabled)
    # Enable Robot motion
    robot_motion_enabled = Bool()
    robot_motion_enabled.data = True
    self.robot_motion_enabled_pub.publish(robot_motion_enabled)

    # Change status
    self.setRobotStatus(self.robot_status_types['FLYING'])

    return


  def setRobotLanding(self):

    # Disable controls
    robot_cmd_control_enabled = Bool()
    robot_cmd_control_enabled.data = False
    self.robot_cmd_control_enabled_pub.publish(robot_cmd_control_enabled)
    # Disable Robot motion
    robot_motion_enabled = Bool()
    robot_motion_enabled.data = False
    self.robot_motion_enabled_pub.publish(robot_motion_enabled)

    # Change status
    self.setRobotStatus(self.robot_status_types['LANDING'])

    return


  def setRobotTakingOff(self):

    # Disable controls (just in case)
    robot_cmd_control_enabled = Bool()
    robot_cmd_control_enabled.data = False
    self.robot_cmd_control_enabled_pub.publish(robot_cmd_control_enabled)
    # Disable Robot motion (just in case)
    robot_motion_enabled = Bool()
    robot_motion_enabled.data = False
    self.robot_motion_enabled_pub.publish(robot_motion_enabled)

    # Change status
    self.setRobotStatus(self.robot_status_types['TAKING_OFF'])


  def setRobotStatus(self, robot_status_type):

    self.robot_status = robot_status_type


  def getRobotStatus(self):

    return self.robot_status


  def take_off(self):

    #
    print("Take off command")

    # Check
    if(self.robot_status != self.robot_status_types['LANDED']):
      print("Cannot take off")
      return False

    #
    print("Taking off")

    #
    self.setRobotTakingOff()

    # Start timer
    self.taking_off_timer = rospy.Timer(rospy.Duration(self.taking_off_duration), self.takingOffTimerCallback, oneshot=True)
      
    # End
    return True


  def land(self):

    #
    print("Land command")

    # Check
    if(self.robot_status != self.robot_status_types['FLYING']):
      print("Cannot land")
      return False

    #
    print("Landing")

    #
    self.setRobotLanding()

    # Start timer
    self.landing_timer = rospy.Timer(rospy.Duration(self.landing_duration), self.landingTimerCallback, oneshot=True)
    
    # End
    return True


  def takingOffTimerCallback(self, event):

    # Check
    if(self.robot_status != self.robot_status_types['TAKING_OFF']):
      print("Error taking off")
      return

    #
    print("Flying")

    #
    self.setRobotFlying()

    return


  def landingTimerCallback(self, event):

    # Check
    if(self.robot_status != self.robot_status_types['LANDING']):
      print("Error landing")
      return

    #
    print("Landed")

    # Change status
    self.setRobotStatus(self.robot_status_types['LANDED'])

    return


  def takeoffCallback(self, msg):

    self.take_off()

    return


  def landCallback(self, msg):

    self.land()

    return


  def robotStatusTimerCallback(self, event):

    robot_status_msgs = RobotStatus()

    if(self.robot_status == self.robot_status_types['FLYING']):
      # Flying
      robot_status_msgs.robot_status = robot_status_msgs.FLYING
    elif(self.robot_status == self.robot_status_types['LANDED']):
      # Landed
      robot_status_msgs.robot_status = robot_status_msgs.LANDED
    elif(self.robot_status == self.robot_status_types['TAKING_OFF']):
      # Taking-off
      robot_status_msgs.robot_status = robot_status_msgs.TAKING_OFF
    elif(self.robot_status == self.robot_status_types['LANDING']):
      # Landing
      robot_status_msgs.robot_status = robot_status_msgs.LANDING
    else:
      # Unknown
      robot_status_msgs.robot_status = robot_status_msgs.UNKNOWN

    # Publish
    self.robot_status_pub.publish(robot_status_msgs)


    return

