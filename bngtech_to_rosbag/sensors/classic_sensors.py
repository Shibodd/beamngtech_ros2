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
      k: parser(self.typestore, time, self.vehicle.sensors[k])
      for k, parser in self.sensors.items()
    }
  
import numpy as np

def vehicle_state_odometry(typestore: Typestore, time, sample):
  ns, stamp = msg_helpers.stamp_from_float(time, typestore)

  return typestore.types['nav_msgs/msg/Odometry'](
    child_frame_id = 'map',
    header = msg_helpers.header(typestore, 'map', stamp),
    pose = typestore.types['geometry_msgs/msg/PoseWithCovariance'](
      pose = typestore.types['geometry_msgs/msg/Pose'](
        position = msg_helpers.point(typestore, sample['pos']),
        orientation = msg_helpers.quaternion(typestore, sample['rotation'])
      ),
      covariance = np.zeros((36,), dtype=np.float64)
    ),
    twist = typestore.types['geometry_msgs/msg/TwistWithCovariance'](
      twist = typestore.types['geometry_msgs/msg/Twist'](
        linear = msg_helpers.vector3(typestore, sample['vel']),
        angular = msg_helpers.vector3(typestore, [0,0,0])
      ),
      covariance = np.zeros((36,), dtype=np.float64)
    )
  )