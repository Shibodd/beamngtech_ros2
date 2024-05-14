from .factories import factory
import rclpy.node
import rclpy.parameter
import beamngpy.sensors
from utils.param_parser import ParameterParser, ParameterType
import utils.geometry_helpers as geometry_helpers

import numpy as np
from dataclasses import dataclass

FACTORY_MAP = {}

def bng_poll(sensor):
  data = sensor.poll()
  if isinstance(data, dict):
    return list(data.values())
  return []

@factory('gps', FACTORY_MAP)
class GPSSensor:
  def __init__(self, node: rclpy.node.Node, prefix: str):
    params = ParameterParser(node, prefix)
    self.name = params.expected('name', ParameterType.PARAMETER_STRING)
    self.sensor = beamngpy.sensors.GPS(
      self.name,
      node.bng,
      node.vehicle,
      physics_update_time=params.optional('physics_update_time', default=0.1),
      pos=params.optional('pos', rclpy.parameter.array.array('d', [0.0, 0.0, 1.7])),
      ref_lon=params.optional('ref_lon', 0.0),
      ref_lat=params.optional('ref_lat', 0.0)
    )

  def poll(self, workspace: dict):
    workspace[self.name] = [
      {
        'time': sample['time'],
        'latitude': sample['lat'],
        'longitude': sample['lon']
      }
      for sample in bng_poll(self.sensor)
    ]
  
  def remove(self):
    self.sensor.remove()

@factory('imu', FACTORY_MAP)
class IMUSensor:
  @dataclass
  class Sample:
    time: float
    position: np.ndarray[np.float64]
    orientation: np.ndarray[np.float64]
    linear_acceleration: np.ndarray[np.float64]
    angular_velocity: np.ndarray[np.float64]

  def __init__(self, node: rclpy.node.Node, prefix: str):
    params = ParameterParser(node, prefix)
    self.name = params.expected('name', ParameterType.PARAMETER_STRING)
    self.sensor = beamngpy.sensors.AdvancedIMU(
      self.name,
      node.bng,
      node.vehicle,
      physics_update_time=params.optional('physics_update_time', 0.003),
      pos=params.optional('pos', rclpy.parameter.array.array('d', [0.0, 0.0, 1.7])),
      dir=params.optional('dir', rclpy.parameter.array.array('d', [0.0, -1.0, 0.0])),
      up=params.optional('up', rclpy.parameter.array.array('d', [1.0, 0.0, 0.0])),
      is_using_gravity=params.optional('is_using_gravity', False),
      accel_frequency_cutoff=params.optional('accel_frequency_cutoff', None, type=ParameterType.PARAMETER_DOUBLE),
      accel_window_width=params.optional('accel_window_width', None, type=ParameterType.PARAMETER_DOUBLE),
      gyro_frequency_cutoff=params.optional('gyro_frequency_cutoff', None, type=ParameterType.PARAMETER_DOUBLE),
      gyro_window_width=params.optional('gyro_window_width', None, type=ParameterType.PARAMETER_DOUBLE),
    )

  def poll(self, workspace: dict):
    workspace[self.name] = [
      {
        'time': sample['time'],
        'position': np.array(sample['pos']).reshape((3,)),
        'orientation': geometry_helpers.quat_from_axes(sample['dirX'], sample['dirY'], sample['dirZ']),
        'linear_acceleration': np.array(sample['accSmooth']).reshape((3,)),
        'angular_velocity': np.array(sample['angVel']).reshape((3,))
      }
      for sample in bng_poll(self.sensor)
    ]

  def remove(self):
    self.sensor.remove()


@factory('powertrain', FACTORY_MAP)
class PowertrainSensor:
  def __init__(self, node: rclpy.node.Node, prefix: str):
    self.sensor = beamngpy.sensors.PowertrainSensor("powertrain", node.bng, node.vehicle)
  
  def poll(self, workspace: dict):
    pass