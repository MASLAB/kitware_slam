# Messages
from kitware_interface.msg import DriveCmd
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
# Node
import rclpy
from rclpy.node import Node

# Math
from math import pi, sqrt, atan2, asin

# Utilities
from .transformation import euler_from_quaternion
from .practical_pid import PracticalPID

# Update rate
RATE = 100

# PID constants
## Distance
D_P = 0.7
D_I = 0.1
D_D = 0.5
D_MAX = 0.4
D_MID = 0.3
D_MIN = 0.1
## Angle
A_P = 1.0
A_I = 0.1
A_D = 1.0
A_MAX = 0.15
A_MID = 0.1
A_MIN = 0.0

# Error Tolerance
## Distance
D_TOL = 0.025 # meter
## Angle
A_TOL = 0.03 # radian

class DifferentialDriverNode(Node):
  def __init__(self):
    super().__init__('differential_driver')
    self.get_logger().info('Starting differential wheel node')

    # Current position
    self.x_current = 0.0
    self.y_current = 0.0
    self.a_current = 0.0 # radian
    # Desired position
    self.x_desired = 0.0
    self.y_desired = 0.0
    self.a_desired = 0.0 # radian

    # State machine
    self.state = 'rest'

    # PID controllers
    timer_period = 1.0 / RATE
    self.distance_pid = PracticalPID(D_P, D_I, D_D, D_MAX, D_MID, D_MIN, timer_period)
    self.angle_pid = PracticalPID(A_P, A_I, A_D, A_MAX, A_MID, A_MIN, timer_period)

    # Drive publisher
    self.drive_command_publisher = self.create_publisher(DriveCmd, 'drive_cmd', 10)
    self.drive_command = DriveCmd()
    self.__set_speed(0, 0)

    # Subscribers
    self.current_position_subscriber = self.create_subscription(Odometry, 'odom', self.position_callback, 10)
    self.goal_position_subscriber = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
    
    # Timer for advancing state machine
    self.timer = self.create_timer(timer_period, self.timer_callback)


  def timer_callback(self):
    if self.state == 'rest':
      pass # Do nothing
      
    elif self.state == 'rotate_to_goal_init': # From goal_callback
      print('Rotating to goal')
      self.__reset()
      self.state = 'rotate_to_goal'

    elif self.state == 'rotate_to_goal':
      # Calculate error
      (d_error, a_error) = self.__get_error()
      # Determine angle tolerance based on distance tolerance sine error
      a_tol = asin(D_TOL, d_error) 
      # Update gain
      (a_gain, a_done) = self.angle_pid.update(a_error, a_tol)

      if a_done:
        self.__set_speed(0, 0)
        self.state = 'straight_to_goal_init'
      else:
        left_speed = a_gain
        right_speed = -a_gain
        self.__set_speed(left_speed, right_speed)
    
    elif self.state == 'straight_to_goal_init':
      print('Go straight to goal')
      self.__reset()
      self.state = 'straight_to_goal'
    
    elif self.state == 'straight_to_goal':
      # Calculate error
      (d_error, a_error) = self.__get_error()
      # Determine angle tolerance based on distance tolerance sine error
      a_tol = asin(D_TOL, d_error) 
      # Update gains
      (d_gain, d_done) = self.distance_pid.update(d_error, D_TOL)
      (a_gain, a_done) = self.angle_pid.update(a_error, a_tol)
      
      if d_done:
        self.__set_speed(0, 0)
        self.state = 'rotate_to_heading_init'
      else:
        left_speed = d_gain + a_gain
        right_speed = d_gain - a_gain
        self.__set_speed(left_speed, right_speed)
    
    elif self.state == 'rotate_to_heading_init':
      print('Rotate to heading')
      self.__reset()
      self.state = 'rotate_to_heading'
    
    elif self.state == 'rotate_to_heading':
      # Calculate error
      (d_error, a_error) = self.__get_error(to_goal_heading=True)
      # Update gains
      (a_gain, a_done) = self.angle_pid.update(a_error, A_TOL)
      
      if a_done:
        self.__set_speed(0, 0)
        self.state = 'rest'
        print('Finish')
      else:
        left_speed = a_gain
        right_speed = -a_gain
        self.__set_speed(left_speed, right_speed)


  def position_callback(self, msg: Odometry):
    pose = msg.pose.pose
    self.x_current = pose.position.x
    self.y_current = pose.position.y
    self.a_current = euler_from_quaternion(pose.orientation)[2]

  
  def goal_callback(self, msg: PoseStamped):
    print('New goal set')
    # Set new desired location
    pose = msg.pose
    self.x_desired = pose.position.x
    self.y_desired = pose.position.y
    self.a_desired = euler_from_quaternion(pose.orientation)[2]

    # Start rotating
    self.state = 'rotate_to_goal_init'


  def __reset(self):
    self.distance_pid.reset()
    self.angle_pid.reset()


  @staticmethod
  def __correct_angle(angle):
    '''
    Correct angle to between pi and -pi
    '''
    if angle > pi:
      return angle - 2*pi
    elif angle < -pi:
      return angle + 2*pi
    else:
      return angle
    
  
  def __get_error(self, to_goal_heading=False):
    '''
    Get angle and distance error based on current readings.
    Angle error is relative to the set goal heading if 
    to_goal_heading is True, else angle error is error
    between the current robot heading and the angle toward
    the set goal
    '''
    x_error = self.x_desired - self.x_current
    y_error = self.y_desired - self.y_current
    d_error = sqrt(x_error**2 + y_error**2)
    a_desired = self.a_desired if to_goal_heading else atan2(y_error, x_error)
    a_error = self.__correct_angle(a_desired - self.a_current)
    if abs(a_error) > pi/2: # Make error negative if the goal is behind
      d_error = -d_error
    return (d_error, a_error)
  

  def __set_speed(self, left_speed, right_speed): # positive speed means forward
    self.drive_command.l_speed = float(left_speed)
    self.drive_command.r_speed = float(right_speed)
    self.drive_command_publisher.publish(self.drive_command)

def main():
  rclpy.init()
  node = DifferentialDriverNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()