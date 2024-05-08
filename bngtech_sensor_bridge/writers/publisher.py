import rclpy.node
import factories.outputs
import utils.qos

class MessagePublisher:
  def __init__(self, node: rclpy.node.Node, outputs: list[factories.outputs.OutputTopic]):
    self.node = node
    self.publishers = {}
    for output in outputs:
      self.publishers[output.topic] = self.node.create_publisher(
        output.msg_type,
        output.topic,
        utils.qos.QOS_MAP[output.qos] if output.qos else 10
      )

  def write(self, data):
    for topic, msgs in data.items():
      for _, msg in msgs:
        self.publishers[topic].publish(msg)