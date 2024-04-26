import beamngpy.sensors
import numpy as np
from rosbags.typesys.store import Typestore
from . import msg_helpers
from .geometry_helpers import quat_from_axes

class AdvancedIMUSensor:
  def __init__(self, typestore: Typestore, sensor_name, IMU: beamngpy.sensors.AdvancedIMU):
    self.typestore = typestore
    self.sensor_name = sensor_name
    self.IMU = IMU

  def parse_sample(self, sample):
    ns, stamp = msg_helpers.stamp_from_float(sample['time'], self.typestore)

    return ns, self.typestore.types['sensor_msgs/msg/Imu'](
      header = msg_helpers.header(self.typestore, 'imu_link', stamp),
      orientation = msg_helpers.quaternion(self.typestore,
        quat_from_axes(sample['dirX'], sample['dirY'], sample['dirZ'])
      ),
      angular_velocity = msg_helpers.vector3(self.typestore, sample['angVel']),
      linear_acceleration = msg_helpers.vector3(self.typestore, sample['accRaw']),
      orientation_covariance = np.zeros((9, ), dtype=np.float64),
      angular_velocity_covariance = np.zeros((9, ), dtype=np.float64),
      linear_acceleration_covariance = np.zeros((9, ), dtype=np.float64)
    )

  def extract_pose(self, msgs):
    return [ (ns, self.typestore.types['geometry_msgs/msg/PoseStamped'](
      header = msg_helpers.header(self.typestore, 'world', msg.header.stamp),
      pose = self.typestore.types['geometry_msgs/msg/Pose'](
        position = msg_helpers.vector3(self.typestore, [0,0,0]),
        orientation = msg.orientation
      )
    )) for ns, msg in msgs ]

  def poll_msgs(self):
    data = self.IMU.poll()
    if not isinstance(data, dict):
      return { self.sensor_name: [] }
    
    msgs = [self.parse_sample(x) for x in data.values()]
    return {
      self.sensor_name: msgs,
      self.sensor_name + '_pose': self.extract_pose(msgs)
    }