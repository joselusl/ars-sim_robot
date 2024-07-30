import rclpy
from rclpy.node import Node

import std_msgs.msg
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Header

from ars_robot_msgs.msg import RobotStatus



class ArsSimRobotStatusRos(Node):

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
  taking_off_timer_active = False 
  # secs
  landing_duration = 3.0
  landing_timer = None
  landing_timer_active = False

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

  def __init__(self, node_name='ars_sim_robot_status_node'):

    # Init ROS
    super().__init__(node_name)

    #
    self.robot_status = self.robot_status_types['UNKNOWN']

    self.__init(node_name)

    return


  def __init(self, node_name='ars_sim_robot_status_node'):

    #### READING PARAMETERS ###
    self.declare_parameter('robot_init_status_flying', False)
    self.robot_init_status_flying = self.get_parameter('robot_init_status_flying').get_parameter_value().bool_value
    

    # End
    return


  def open(self):

    # Publishers
    #
    self.robot_status_pub = self.create_publisher(RobotStatus, "robot_status", qos_profile=10)
    # Robot cmd control enabled
    robot_cmd_control_enabled_qos_profile = rclpy.qos.QoSProfile(depth=1)
    robot_cmd_control_enabled_qos_profile.history=rclpy.qos.HistoryPolicy.KEEP_LAST
    robot_cmd_control_enabled_qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    robot_cmd_control_enabled_qos_profile.reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
    self.robot_cmd_control_enabled_pub = self.create_publisher(Bool, 'robot_cmd_control_enabled', robot_cmd_control_enabled_qos_profile)
    # Robot motion enabled
    robot_motion_enabled_qos_profile = rclpy.qos.QoSProfile(depth=1)
    robot_motion_enabled_qos_profile.history=rclpy.qos.HistoryPolicy.KEEP_LAST
    robot_motion_enabled_qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    robot_motion_enabled_qos_profile.reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
    self.robot_motion_enabled_pub = self.create_publisher(Bool, 'robot_motion_enabled', robot_motion_enabled_qos_profile)


    # Subscribers
    #
    self.robot_takeoff_sub = self.create_subscription(Empty, "takeoff", self.takeoffCallback, qos_profile=10)
    #
    self.robot_land_sub = self.create_subscription(Empty, "land", self.landCallback, qos_profile=10)


    # Timers
    self.robot_status_timer = self.create_timer(0.1, self.robotStatusTimerCallback)



    # Init
    if(self.robot_init_status_flying is True):
      self.get_logger().info('Flying')
      self.setRobotFlying()
    else:
      self.get_logger().info('Landed')
      self.setRobotLanded()


    # End
    return


  def run(self):

    rclpy.spin(self)

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
    self.get_logger().info("Take off command")

    # Check
    if(self.robot_status == self.robot_status_types['TAKING_OFF']):
      self.get_logger().info("-Already taking off")
      return False
    if(self.robot_status == self.robot_status_types['FLYING']):
      self.get_logger().info("-Already flying")
      return False
    if(self.robot_status != self.robot_status_types['LANDED'] and self.robot_status != self.robot_status_types['TAKING_OFF'] and self.robot_status != self.robot_status_types['FLYING']):
      self.get_logger().info("-Cannot take off")
      return False

    #
    self.get_logger().info("Taking off")

    #
    self.setRobotTakingOff()

    # Start timer
    self.taking_off_timer = self.create_timer(self.taking_off_duration, self.takingOffTimerCallback)
    self.taking_off_timer_active = True 

    # End
    return True


  def land(self):

    #
    self.get_logger().info("Land command")

    # Check
    if(self.robot_status == self.robot_status_types['LANDING']):
      self.get_logger().info("-Already landing")
      return False
    if(self.robot_status == self.robot_status_types['LANDED']):
      self.get_logger().info("-Already landed")
      return False
    if(self.robot_status != self.robot_status_types['FLYING'] and self.robot_status != self.robot_status_types['LANDING'] and self.robot_status != self.robot_status_types['LANDED']):
      self.get_logger().info("-Cannot land")
      return False

    #
    self.get_logger().info("Landing")

    #
    self.setRobotLanding()

    # Start timer
    self.landing_timer = self.create_timer(self.landing_duration, self.landingTimerCallback)
    self.landing_timer_active = True 
    
    # End
    return True


  def takingOffTimerCallback(self):

    if self.taking_off_timer_active:

      # Check
      if(self.robot_status != self.robot_status_types['TAKING_OFF']):
        self.get_logger().info("Error taking off")
        return

      #
      self.get_logger().info("Flying")

      #
      self.setRobotFlying()

      # Deactivate the timer
      self.taking_off_timer_active = False
      self.taking_off_timer.cancel()
      

    return


  def landingTimerCallback(self):

    if self.landing_timer_active:

      # Check
      if(self.robot_status != self.robot_status_types['LANDING']):
        self.get_logger().info("Error landing")
        return

      #
      self.get_logger().info("Landed")

      # Change status
      self.setRobotLanded()

      # Deactivate the timer
      self.landing_timer_active = False
      self.landing_timer.cancel()

    return


  def takeoffCallback(self, msg):

    self.take_off()

    return


  def landCallback(self, msg):

    self.land()

    return


  def robotStatusTimerCallback(self):

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

