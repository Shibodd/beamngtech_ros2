import beamngpy.sensors
import numpy as np
from rosbags.typesys.store import Typestore
import scipy.spatial.transform
import msg_helpers

def quat_from_axes(x, y, z):
  rot = np.hstack((
    np.array(x).reshape(3, 1),
    np.array(y).reshape(3, 1),
    np.array(z).reshape(3, 1)
  ))
  return scipy.spatial.transform.Rotation.from_matrix(rot).as_quat()


class AdvancedIMUSensor:
  def __init__(self, typestore: Typestore, IMU: beamngpy.sensors.AdvancedIMU):
    self.typestore = typestore
    self.IMU = IMU

  def parse_sample(self, sample):
    ns, stamp = msg_helpers.stamp_from_float(sample['time'], self.typestore)

    return ns, self.typestore.types['sensor_msgs/msg/Imu'](
      header = msg_helpers.header(self.typestore, 'map', stamp),
      orientation = msg_helpers.quaternion(self.typestore,
        quat_from_axes(sample['dirX'], sample['dirY'], sample['dirZ'])
      ),
      angular_velocity = msg_helpers.quaternion(self.typestore, sample['orientation']),
      linear_acceleration = msg_helpers.vector3(self.typestore, sample['accRaw']),
      orientation_covariance = np.zeros((9, ), dtype=np.float64),
      angular_velocity_covariance = np.zeros((9, ), dtype=np.float64),
      linear_acceleration_covariance = np.zeros((9, ), dtype=np.float64)
    )

  def poll_msgs(self):
    data = self.IMU.poll()
    if not isinstance(data, dict):
      return []
    return [self.parse_sample(x) for x in data.values()]