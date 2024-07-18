#!/usr/bin/env python

import rclpy
from rclpy.node import Node

import numpy as np
from numpy import *

import os


import rospy


from ars_sim_robot_dynamics_ros import *




def main(args=None):

  rclpy.init(args=args)

  ars_sim_robot_dynamics_ros = ArsSimRobotDynamicsRos()

  ars_sim_robot_dynamics_ros.init()
  ars_sim_robot_dynamics_ros.open()

  try:
    ars_sim_robot_dynamics_ros.run()
  except rospy.ROSInterruptException:
    pass


  minimal_publisher.destroy_node()
  rclpy.shutdown()


  return 0



''' MAIN '''
if __name__ == '__main__':

  main()