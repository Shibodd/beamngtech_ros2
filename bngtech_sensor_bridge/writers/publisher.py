import rclpy.node

class MessagePublisher:
  def __init__(self, node: rclpy.node.Node):
    self.node = node
    self.publishers = {}

  def write(self, data):
    for topic, msgs in data.items():
      if len(msgs) <= 0:
        continue
      
      reg = self.publishers.get(topic, None)

      if not reg:
        typ = type(msgs[0][1])
        pub = self.node.create_publisher(typ, topic, 10)
        self.publishers[topic] = (typ, pub)
      else:
        typ, pub = reg

      for _, msg in msgs:
        assert type(msg) == typ
        pub.publish(msg)