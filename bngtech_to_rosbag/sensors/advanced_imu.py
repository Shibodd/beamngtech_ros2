import beamngpy.sensors
from rosbags.typesys.store import Typestore
from . import geometry_helpers
from . import msg_iface

class AdvancedIMUSensor:
  def __init__(self, typestore: Typestore, sensor_name, IMU: beamngpy.sensors.AdvancedIMU):
    self.typestore = typestore
    self.sensor_name = sensor_name
    self.IMU = IMU

  def parse_sample(self, time, sample):
    return msg_iface.ImuData(
      time = time,
      frame = 'imu_link',
      orientation = msg_iface.Vector4Covariance3Pair.ground_truth(geometry_helpers.quat_from_axes(sample['dirX'], sample['dirY'], sample['dirZ'])),
      angular_velocity = msg_iface.Vector3Covariance3Pair.ground_truth(sample['angVel']),
      linear_acceleration = msg_iface.Vector3Covariance3Pair.ground_truth(sample['accSmooth'])
    )

  def poll_data(self):
    data = self.IMU.poll()
    if not isinstance(data, dict):
      return { self.sensor_name: [] }
    
    return {
      self.sensor_name: [ self.parse_sample(x['time'], x) for x in data.values()]
    }