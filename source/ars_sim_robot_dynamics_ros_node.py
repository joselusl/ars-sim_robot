#!/usr/bin/env python

import numpy as np
from numpy import *

import os


import rospy


from ars_sim_robot_dynamics_ros import *




def main():

  ars_sim_robot_dynamics_ros = ArsSimRobotDynamicsRos()

  ars_sim_robot_dynamics_ros.init()
  ars_sim_robot_dynamics_ros.open()

  try:
    ars_sim_robot_dynamics_ros.run()
  except rospy.ROSInterruptException:
    pass


  return 0



''' MAIN '''
if __name__ == '__main__':

  main()