import itertools
import rcl_interfaces.msg
import rclpy.parameter

def factory(type, map):
  def wrapper(fn):
    assert type not in map
    map[type] = fn
    return fn
  return wrapper

def parse_list(node, list_prefix, type_map):
  pattern = f'{list_prefix}._%d'

  ans = []
  for i in itertools.count():
    elem_prefix = pattern % i
    node.declare_parameter(f"{elem_prefix}.type", '')
    output_type = node.get_parameter(f"{elem_prefix}.type").value
    if not output_type:
      return ans
    ans.append(type_map[output_type](node, elem_prefix))
  assert False

def parse_simple_list(node, list_name, type_map):
  names = node.declare_parameter(list_name, descriptor=rcl_interfaces.msg.ParameterDescriptor(type=rclpy.parameter.ParameterType.PARAMETER_STRING_ARRAY)).value
  names = list(names) if names is not None else []
  return [type_map[name](node) for name in names]