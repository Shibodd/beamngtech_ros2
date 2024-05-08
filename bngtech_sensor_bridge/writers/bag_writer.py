import rosbags.rosbag2 as rb2
import rosbags.typesys

import utils.qos
import factories.outputs
import itertools

import rclpy.node
import utils.param_parser

class BagWriter:
  typestore = rosbags.typesys.get_typestore(rosbags.typesys.Stores.ROS2_HUMBLE)

  def __init__(self, node: rclpy.node.Node, outputs: list[factories.outputs.OutputTopic]):
    params = utils.param_parser.ParameterParser(node, 'bag_writer')
    path = params.optional('path', None, utils.param_parser.ParameterType.PARAMETER_STRING)
    
    if path is not None:
      self.writer = rb2.Writer(path)
      self.writer.open()
      self.connections = {}
      for output in outputs:
        qos = utils.qos.qos_to_yaml(utils.qos.QOS_MAP[output.qos]) if output.qos else ''
        msg_type = "/".join(
          itertools.chain(
            output.msg_type.__module__.split(".")[:-1], 
            (output.msg_type.__name__, )
          )
        )

        self.connections[output.topic] = self.writer.add_connection(
          output.topic,
          msg_type,
          typestore=self.typestore,
          offered_qos_profiles=qos
        )

  def write(self, data):
    if hasattr(self, 'writer'):
      for topic, msgs in data.items():
        for t, msg in msgs:
          conn = self.connections[topic]
          self.writer.write(conn, int(t * 10**9), self.typestore.serialize_cdr(msg, conn.msgtype))

  def close(self):
    if hasattr(self, 'writer'):
      self.writer.close()