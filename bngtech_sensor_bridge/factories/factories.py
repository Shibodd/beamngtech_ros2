import itertools

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