import rclpy.duration
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
import rclpy
import yaml

qos = QoSProfile(
  depth=1,
  durability=DurabilityPolicy.TRANSIENT_LOCAL,
  history=HistoryPolicy.KEEP_LAST,
)
qos = qos.get_c_qos_profile().to_dict()
for k, v in qos.items():
  if isinstance(v, rclpy.duration.Duration):
    qos[k] = {
      'sec': v.nanoseconds // 10**9,
      'nsec': v.nanoseconds % 10**9
    }

TRANSIENT_LOCAL_QOS = yaml.dump([qos])
del qos