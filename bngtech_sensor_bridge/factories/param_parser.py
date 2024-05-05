import rcl_interfaces.msg
import rclpy.parameter
from rclpy.parameter import ParameterType
import rclpy.node

class MissingParameterException(Exception):
  pass


class ParameterParser:
  def __init__(self, node: rclpy.node.Node, prefix: str):
    self.node = node
    self.prefix = prefix

  def expected(self, name, type: ParameterType):
    ans = self.optional(name, None, type=type)
    if ans is None:
      self.node.get_logger().fatal(f'Missing expected parameter "{self.prefix}.{name}".')
      raise MissingParameterException()
    return ans

  def optional(self, name, default, type=None):
    assert default is not None or type is not None

    parameter = f'{self.prefix}.{name}'
    if self.node.has_parameter(parameter):
      return self.node.get_parameter(parameter).value

    return self.node.declare_parameter(
      parameter,
      default,
      descriptor=rcl_interfaces.msg.ParameterDescriptor(type=type) if type is not None else None
    ).value