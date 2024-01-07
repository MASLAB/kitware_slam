from geometry_msgs.msg import Quaternion

from math import atan2, asin, sin, cos

# Adapted from https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
def euler_from_quaternion(quaternion : Quaternion):
  """
  Converts Quaternion to Euler roll, pitch, yaw
  """
  x = quaternion.x
  y = quaternion.y
  z = quaternion.z
  w = quaternion.w

  sinr_cosp = 2 * (w * x + y * z)
  cosr_cosp = 1 - 2 * (x * x + y * y)
  roll = atan2(sinr_cosp, cosr_cosp)

  sinp = 2 * (w * y - z * x)
  pitch = asin(sinp)

  siny_cosp = 2 * (w * z + x * y)
  cosy_cosp = 1 - 2 * (y * y + z * z)
  yaw = atan2(siny_cosp, cosy_cosp)

  return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
  """
  Converts Euler roll, pitch, yaw to Quaternion
  """
  cy = cos(yaw * 0.5)
  sy = sin(yaw * 0.5)
  cp = cos(pitch * 0.5)
  sp = sin(pitch * 0.5)
  cr = cos(roll * 0.5)
  sr = sin(roll * 0.5)

  q = Quaternion()
  q.w = cy * cp * cr + sy * sp * sr
  q.x = cy * cp * sr - sy * sp * cr
  q.y = sy * cp * sr + cy * sp * cr
  q.z = sy * cp * cr - cy * sp * sr

  return q 