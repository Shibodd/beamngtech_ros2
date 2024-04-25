from rosbags.typesys.store import Typestore
from . import msg_helpers

class ClassicSensors:
  def __init__(self, typestore, vehicle, sensors: dict, timer_name: str):
    self.vehicle = vehicle
    self.sensors = sensors
    self.timer_name = timer_name
    self.typestore = typestore

  def poll_msgs(self):
    self.vehicle.sensors.poll()
    time = self.vehicle.sensors[self.timer_name]['time']
    return {
      k: [ npp[1](self.typestore, time, self.vehicle.sensors[npp[0]]) ]
      for k, npp in self.sensors.items()
    }

def odometry_pose(typestore: Typestore, time, sample):
  ns, stamp = msg_helpers.stamp_from_float(time, typestore)

  return ns, typestore.types['geometry_msgs/msg/PoseStamped'](
    header = msg_helpers.header(typestore, 'track', stamp),
    pose = typestore.types['geometry_msgs/msg/Pose'](
      position = msg_helpers.point(typestore, sample['pos']),
      orientation = msg_helpers.quaternion(typestore, sample['rotation'])
    )
  )

def twist_wheelspeed(typestore: Typestore, time, sample):
  ns, stamp = msg_helpers.stamp_from_float(time, typestore)
  return ns, typestore.types['geometry_msgs/msg/TwistStamped'](
    header = msg_helpers.header(typestore, 'imu_link', stamp),
    twist = typestore.types['geometry_msgs/msg/Twist'](
      linear = msg_helpers.vector3(typestore, [sample['wheelspeed'], 0, 0]),
      angular = msg_helpers.vector3(typestore, [0, 0, 0])
    ),
  )