#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg







class ArsSimRobotRos:

  

  def __init__(self):

    return


  def init(self, node_name='ars_sim_robot_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_sim_robot')
    

    #### READING PARAMETERS ###
    
    
    return


  def open(self):


    return


  def run(self):

    rospy.spin()

    return


  