from . import geometry_helpers
from . import msg_iface
import numpy as np

class ClassicSensors:
  def __init__(self, typestore, vehicle, sensors: dict, timer_name: str):
    self.vehicle = vehicle
    self.sensors = sensors
    self.timer_name = timer_name
    self.typestore = typestore
    self.last_timestamp = {}
    self.start_position = None

  def poll_data(self):
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
      return [ parser(time, self.vehicle.sensors[bng_sensor_name]) ]
    return []

def odometry_pose(time, sample):
  return msg_iface.PoseData(
    time = time,
    frame = 'track',
    position = msg_iface.Vector3Covariance3Pair.ground_truth(sample['pos']),
    orientation = msg_iface.Vector4Covariance3Pair.ground_truth(geometry_helpers.quat_from_fwd_up(sample['dir'], sample['up']))
  )

def vehicle_tf(time, sample):
  return msg_iface.TransformData(
    time = time,
    frame = 'track',
    child_frame = 'imu_link',
    translation = np.array(sample['pos'], dtype=np.float64),
    rotation = geometry_helpers.quat_from_fwd_up(sample['dir'], sample['up'])
  )

def twist_wheelspeed(time, sample):
  return msg_iface.TwistData(
    time = time,
    frame = 'imu_link',
    linear = msg_iface.Vector3Covariance3Pair.ground_truth([sample['wheelspeed'], 0, 0]),
    angular = msg_iface.Vector3Covariance3Pair.ground_truth([0,0,0])
  )