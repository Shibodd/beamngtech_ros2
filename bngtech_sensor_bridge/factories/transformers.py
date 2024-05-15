import rclpy.node
import rclpy.parameter
from utils.param_parser import ParameterParser, ParameterType
from .factories import factory
import numpy as np
from utils.map_to_track_transform import MapToTrackTransform
import utils.geometry_helpers as geometry_helpers

FACTORY_MAP = {}

@factory('map_to_track', FACTORY_MAP)
class MapToTrackTransformer:
  def __init__(self, node, prefix):
    params = ParameterParser(node, prefix)

    def parse_prefix_fmt(name):
      prefix = params.optional(name, '').strip()
      return f"{prefix}.%s" if prefix else "%s"

    self.pose_source = params.expected('pose_source', ParameterType.PARAMETER_STRING)
    self.tf_destination = params.expected('tf_destination', ParameterType.PARAMETER_STRING)
    self.in_fmt = parse_prefix_fmt('in_prefix')
    self.out_fmt = parse_prefix_fmt('out_prefix')

    self.apply_to = list(
      self.__parse_apply_to(x) for x in
      params.expected('apply_to', ParameterType.PARAMETER_STRING_ARRAY)
    )
    self.transform = None

  def __parse_apply_to(self, apply_to):
    typ, from_to = tuple(x.strip() for x in apply_to.split(":"))
    from_to = tuple(x.strip() for x in from_to.split(">"))

    assert(len(from_to) in (1, 2))

    if len(from_to) == 2:
      src, dst = from_to
    else:
      src = dst = from_to[0]

    return (typ, self.in_fmt % src, self.out_fmt % dst)
  
  APPLY_MAP = {
    'position': MapToTrackTransform.transform_position,
    'orientation': MapToTrackTransform.transform_orientation,
    'vector': MapToTrackTransform.transform_vector,
  }

  def tick(self, workspace):
    if self.transform is None and len(workspace[self.pose_source]) > 0:
      pose = workspace[self.pose_source][-1]
      self.transform = MapToTrackTransform(
        origin=pose['position'],
        orientation=pose['orientation'],
      )
      workspace[self.tf_destination] = [{
        'time': pose['time'],
        'position': pose['position'],
        'orientation': pose['orientation']
      }]
    else:
      workspace[self.tf_destination] = []

    if self.transform is not None:
      for typ, src, dst in self.apply_to:
        data = workspace[src]
        if len(data) > 0:
          workspace[dst] = self.APPLY_MAP[typ](self.transform, data)
    else:
      for _, __, dst in self.apply_to:
        workspace[dst] = []

@factory('copy', FACTORY_MAP)
class CopyTransformer:
  def __init__(self, node, prefix):
    params = ParameterParser(node, prefix)
    self.source = params.expected('source', ParameterType.PARAMETER_STRING)
    self.destination = params.expected('destination', ParameterType.PARAMETER_STRING)

  def tick(self, workspace):
    workspace[self.destination] = workspace[self.source]

@factory('global_to_local_velocity', FACTORY_MAP)
class GlobalToLocalVelocityTransformer:
  def __init__(self, node, prefix):
    params = ParameterParser(node, prefix)
    self.velocity_source = params.expected('velocity_source', ParameterType.PARAMETER_STRING)
    self.orientation_source = params.expected('orientation_source', ParameterType.PARAMETER_STRING)
    self.destination = params.expected('destination', ParameterType.PARAMETER_STRING)
  
  def tick(self, workspace):
    velocity = np.array(workspace[self.velocity_source])
    orientation = np.array(workspace[self.orientation_source])

    mat = geometry_helpers.quat_to_matrix(orientation)
    fwd = mat[:, :, 0]
    left = mat[:, :, 1]
    
    workspace[self.destination] = {}
    workspace[self.destination]['vx'] = np.sum(fwd * velocity, axis=1)
    workspace[self.destination]['vy'] = np.sum(left * velocity, axis=1)