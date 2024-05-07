import builtin_interfaces.msg
import std_msgs.msg
import geometry_msgs.msg

def time(t):
  STONS = 10**9
  ns = int(t * STONS)
  return builtin_interfaces.msg.Time(
    sec = ns // STONS,
    nanosec = ns % STONS
  )

def header(t, frame):
  return std_msgs.msg.Header(
    frame_id = frame,
    stamp = time(t)
  )

def vector3(v):  
  return geometry_msgs.msg.Vector3(x=v[0], y=v[1], z=v[2])

def quaternion(v):  
  return geometry_msgs.msg.Quaternion(x=v[0], y=v[1], z=v[2], w=v[3])

def point(v):
  return geometry_msgs.msg.Point(x=v[0], y=v[1], z=v[2])