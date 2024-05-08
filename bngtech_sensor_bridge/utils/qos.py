import rclpy.duration
from rclpy.qos import HistoryPolicy, DurabilityPolicy, QoSProfile
import yaml

QOS_MAP = {
  'transient_local': QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
  )
}

def qos_to_yaml(qos):
  qos = qos.get_c_qos_profile().to_dict()
  for k, v in qos.items():
    if isinstance(v, rclpy.duration.Duration):
      qos[k] = {
        'sec': v.nanoseconds // 10**9,
        'nsec': v.nanoseconds % 10**9
      }
  return yaml.dump([qos])