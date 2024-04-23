def stamp_from_float(seconds: float, typestore):
  STONS = 10**9

  ns = int(seconds * STONS)
  msg = typestore.types['builtin_interfaces/msg/Time'](
    sec = ns // STONS,
    nanosec = ns % STONS
  )

  return (ns, msg)

def header(typestore, frame, stamp):
  return typestore.types['std_msgs/msg/Header'](
    stamp = stamp, 
    frame_id = frame
  )

def quaternion(typestore, quat):
  return typestore.types['geometry_msgs/msg/Quaternion'](*quat)

def vector3(typestore, vec):
  return typestore.types['geometry_msgs/msg/Vector3'](*vec)

def point(typestore, pt):
  return typestore.types['geometry_msgs/msg/Point'](*pt)