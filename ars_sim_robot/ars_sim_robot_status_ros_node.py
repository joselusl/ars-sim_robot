#!/usr/bin/env python3

import numpy as np
from numpy import *

import os


import rclpy

from ars_sim_robot.ars_sim_robot_status_ros import ArsSimRobotStatusRos



def main(args=None):

  rclpy.init(args=args)

  ars_sim_robot_status_ros = ArsSimRobotStatusRos()

  ars_sim_robot_status_ros.open()

  try:
      ars_sim_robot_status_ros.run()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
      # Graceful shutdown on interruption
      pass
  finally:
    ars_sim_robot_status_ros.destroy_node()
    rclpy.try_shutdown()


  return 0



''' MAIN '''
if __name__ == '__main__':

  main()