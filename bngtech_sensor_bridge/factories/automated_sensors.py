from .factories import factory
import rclpy.node
import rclpy.parameter
import beamngpy.sensors
from .param_parser import ParameterParser, ParameterType

FACTORY_MAP = {}

@factory('gps', FACTORY_MAP)
def gps(node: rclpy.node.Node, prefix: str):
  params = ParameterParser(node, prefix)
  return beamngpy.sensors.GPS(
    params.expected('name', ParameterType.PARAMETER_STRING),
    node.bng,
    node.vehicle,
    physics_update_time=params.optional('physics_update_time', default=0.1),
    pos=params.optional('pos', rclpy.parameter.array.array('d', [0.0, 0.0, 1.7])),
    ref_lon=params.optional('ref_lon', 0.0),
    ref_lat=params.optional('ref_lat', 0.0)
  )

@factory('imu', FACTORY_MAP)
def imu(node: rclpy.node.Node, prefix: str):
  params = ParameterParser(node, prefix)
  return beamngpy.sensors.AdvancedIMU(
    params.expected('name', ParameterType.PARAMETER_STRING),
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