from utils.param_parser import ParameterParser
import rclpy.node

def last_or_none(r):
  return None if len(r) == 0 else r[-1]

class ClassicTimeCompensator:
  def __init__(self, node: rclpy.node.Node):
    params = ParameterParser(node, 'classic_time_compensator')
    self.node = node
    self.alpha = params.optional('alpha', 0.2)
    self.avg_offset = None

  def compensate(self, workspace):
    t_timer = workspace['timer']

    adv_ts = (
      last_or_none(workspace[f"{name}[].time"])
      for name in (x.name for x in self.node.automated_sensors)
    )
    adv_ts = [t for t in adv_ts if t is not None]

    if len(adv_ts) > 0:
      t_advanced = max(adv_ts)
      offset = t_timer - t_advanced
      if self.avg_offset is None:
        self.avg_offset = offset
      else:
        self.avg_offset = offset * self.alpha + self.avg_offset * (1-self.alpha)

    if self.avg_offset is None:
      return

    workspace['timer'] = t_timer - self.avg_offset