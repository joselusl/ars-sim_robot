#!/usr/bin/env python

import numpy as np
from numpy import *

import os


import rospy


from ars_sim_robot_status_ros import *




def main():

  ars_sim_robot_status_ros = ArsSimRobotStatusRos()

  ars_sim_robot_status_ros.init()
  ars_sim_robot_status_ros.open()

  try:
    ars_sim_robot_status_ros.run()
  except rospy.ROSInterruptException:
    pass


  return 0



''' MAIN '''
if __name__ == '__main__':

  main()