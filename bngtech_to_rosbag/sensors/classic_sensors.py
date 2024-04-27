from rosbags.typesys.store import Typestore
from . import msg_helpers
from .geometry_helpers import quat_from_fwd_up

class ClassicSensors:
  def __init__(self, typestore, vehicle, sensors: dict, timer_name: str):
    self.vehicle = vehicle
    self.sensors = sensors
    self.timer_name = timer_name
    self.typestore = typestore
    self.last_timestamp = {}

  def poll_msgs(self):
    self.vehicle.sensors.poll()
    time = self.vehicle.sensors[self.timer_name]['time']
    return {
      k: self._time_stab_poll(time, k, npp[0], npp[1], npp[2])
      for k, npp in self.sensors.items()
    }
  
  def _time_stab_poll(self, time, sensor_name, bng_sensor_name, parser, min_interval):
    poll = sensor_name not in self.last_timestamp
    if not poll:
      poll = time - self.last_timestamp[sensor_name] >= min_interval

    if poll:
      self.last_timestamp[sensor_name] = time
      return [ parser(self.typestore, time, self.vehicle.sensors[bng_sensor_name]) ]
    return []


def odometry_pose(typestore: Typestore, time, sample):
  ns, stamp = msg_helpers.stamp_from_float(time, typestore)

  return ns, msg_helpers.with_covariance(typestore, 'pose',
    msg_helpers.header(typestore, 'world', stamp),
    np.zeros((36,), dtype=np.float64),
    typestore.types['geometry_msgs/msg/Pose'](
      position = msg_helpers.point(typestore, sample['pos']),
      orientation = msg_helpers.quaternion(typestore, quat_from_fwd_up(sample['dir'], sample['up']))
    )
  )

def vehicle_tf(typestore: Typestore, time, sample):
  ns, stamp = msg_helpers.stamp_from_float(time, typestore)

  return ns, typestore.types['tf2_msgs/msg/TFMessage'](
    transforms = [
      typestore.types['geometry_msgs/msg/TransformStamped'](
        header = msg_helpers.header(typestore, 'world', stamp),
        child_frame_id = 'vehicle',
        transform = typestore.types['geometry_msgs/msg/Transform'](
          translation = msg_helpers.vector3(typestore, sample['pos']),
          rotation = msg_helpers.quaternion(typestore, quat_from_fwd_up(sample['dir'], sample['up']))
        )
      )
    ]
  )

import numpy as np

def twist_wheelspeed(typestore: Typestore, time, sample):
  ns, stamp = msg_helpers.stamp_from_float(time, typestore)

  return ns, msg_helpers.with_covariance(typestore, 'twist',
    msg_helpers.header(typestore, 'vehicle', stamp),
    np.zeros((36,), dtype=np.float64),
    typestore.types['geometry_msgs/msg/Twist'](
      linear = msg_helpers.vector3(typestore, [sample['wheelspeed'], 0, 0]),
      angular = msg_helpers.vector3(typestore, [0, 0, 0])
    )
  )