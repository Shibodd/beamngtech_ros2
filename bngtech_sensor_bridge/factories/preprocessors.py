from .factories import factory
import numpy as np

import rclpy.node
import rclpy.parameter
from .param_parser import ParameterParser, ParameterType
import geometry_helpers
import dataclasses

FACTORY_MAP = {}

@dataclasses.dataclass
class Pose:
  time: float
  position: np.ndarray[np.float64]
  orientation: np.ndarray[np.float64]

@factory('state_to_pose', FACTORY_MAP)
class StateToPosePreprocessor:
  def __init__(self, node: rclpy.node.Node, prefix: str):
    params = ParameterParser(node, prefix)
    self.output = params.expected('output', ParameterType.PARAMETER_STRING)
  
  def tick(self, data):
    state = data['state']

    data[self.output] = [
      Pose(
        time = data['timer']['time'],
        position = np.array(state['pos']).reshape((3,)),
        orientation = geometry_helpers.quat_from_fwd_up(state['dir'], state['up'])
      )
    ]

@factory('imu_to_pose', FACTORY_MAP)
class IMUToPosePreprocessor:
  def __init__(self, node: rclpy.node.Node, prefix: str):
    params = ParameterParser(node, prefix)
    self.source = params.expected('source', ParameterType.PARAMETER_STRING)
    self.output = params.expected('output', ParameterType.PARAMETER_STRING)
  
  def tick(self, data):
    data[self.output] = [
      Pose(
        time = sample['time'],
        position = np.array(sample['pos']).reshape((3,)),
        orientation = geometry_helpers.quat_from_fwd_up(sample['dirX'], sample['dirZ'])
      )
      for sample in data[self.source]
    ]

@factory('map_to_track', FACTORY_MAP)
class MapToTrackPreprocessor:
  def __init__(self, node, prefix):
    params = ParameterParser(node, prefix)
    self.source = params.expected('source', ParameterType.PARAMETER_STRING)

    self.apply_to = list(
      self.__parse_apply_to(x) for x in 
      params.expected('apply_to', ParameterType.PARAMETER_STRING_ARRAY)
    )

  def __parse_apply_to(self, x):
    type, name = x.split(":")

    fn = None
    match type:
      case 'pose': fn = self.__apply_to_imu
      case 'imu': fn = self.__apply_to_imu
    assert(fn)
    return (name, fn)

  def __apply_to_pose(self, pose):
    pass

  def __apply_to_imu(self, imu):
    pass

  def tick(self, data):
    for k, fn in self.apply_to:
      fn(data[k])