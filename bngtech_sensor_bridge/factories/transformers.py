import rclpy.node
import rclpy.parameter
from utils.param_parser import ParameterParser, ParameterType
from .factories import factory
import numpy as np
from utils.map_to_track_transform import MapToTrackTransform

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
        workspace[dst] = self.APPLY_MAP[typ](self.transform, workspace[src])
    else:
      for _, dst in self.apply_to:
        workspace[dst] = []