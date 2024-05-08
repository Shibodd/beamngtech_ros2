from .factories import factory
from utils.param_parser import ParameterParser, ParameterType

import rclpy.node
import rclpy.parameter
import numpy as np
import math

import geometry_msgs.msg
import tf2_msgs.msg
import utils.msg_helpers as msg_helpers
import sensor_msgs.msg

FACTORY_MAP = {}

import dataclasses

@dataclasses.dataclass
class OutputTopic:
  topic: str
  msg_type: type
  qos: str

@factory('tf', FACTORY_MAP)
class TFOutput:
  def __init__(self, node: rclpy.node.Node, prefix):
    params = ParameterParser(node, prefix)
    self.topic = params.expected('topic', ParameterType.PARAMETER_STRING)
    self.source = params.expected('source', ParameterType.PARAMETER_STRING)
    self.qos = params.optional('qos', None, ParameterType.PARAMETER_STRING)
    self.frame_id = params.optional('frame_id', 'map')
    self.child_frame_id = params.optional('child_frame_id', 'imu_link')

  def get_topics(self):
    return [
      OutputTopic(self.topic, tf2_msgs.msg.TFMessage, self.qos)
    ]

  def tick(self, workspace):
    return {
      self.topic: [
        (
          sample['time'],
          tf2_msgs.msg.TFMessage(
            transforms = [
              geometry_msgs.msg.TransformStamped(
                header = msg_helpers.header(sample['time'], self.frame_id),
                child_frame_id = self.child_frame_id,
                transform = geometry_msgs.msg.Transform(
                  translation = msg_helpers.vector3(sample['position']),
                  rotation = msg_helpers.quaternion(sample['orientation'])
                )
              )
            ]
          )
        )
        for sample in workspace[self.source]
      ]
    }
  
@factory('wheelspeed', FACTORY_MAP)
class WheelspeedOutput:
  def __init__(self, node: rclpy.node.Node, prefix):
    params = ParameterParser(node, prefix)

    self.topic = params.expected('topic', ParameterType.PARAMETER_STRING)
    self.source = params.expected('source', ParameterType.PARAMETER_STRING)
    self.frame_id = params.optional('frame_id', 'map')
    self.variance = params.optional('variance', 0.0)
    self.qos = params.optional('qos', None, ParameterType.PARAMETER_STRING)

  def get_topics(self):
    return [
      OutputTopic(self.topic, geometry_msgs.msg.TwistWithCovarianceStamped, self.qos)
    ]

  def tick(self, workspace):
    return {
      self.topic: [
        (
          sample['time'],
          geometry_msgs.msg.TwistWithCovarianceStamped(
            header = msg_helpers.header(sample['time'], self.frame_id),
            twist = geometry_msgs.msg.TwistWithCovariance(
              twist = geometry_msgs.msg.Twist(
                linear = msg_helpers.vector3(np.array([sample['wheelspeed'], 0, 0], dtype=np.float64)),
                angular = msg_helpers.vector3(np.zeros((3,), dtype=np.float64))
              ),
              covariance = np.array([self.variance] + [0.0]*35, dtype=np.float64)
            )
          )
        )
        for sample in workspace[self.source]
      ]
    }


@factory('imu', FACTORY_MAP)
class IMUOutput:
  def __init__(self, node: rclpy.node.Node, prefix):
    params = ParameterParser(node, prefix)
    
    self.source = params.expected('source', rclpy.parameter.ParameterType.PARAMETER_STRING)
    self.topic = params.expected('topic', rclpy.parameter.ParameterType.PARAMETER_STRING)
    self.qos = params.optional('qos', None, ParameterType.PARAMETER_STRING)
    self.frame_id = params.optional('frame_id', 'imu_link')
    self.orientation_covariance = np.array(params.optional('orientation_covariance', rclpy.parameter.array.array('d', [0.0]*9)), dtype=np.float64)
    self.angular_velocity_covariance = np.array(params.optional('angular_velocity_covariance', rclpy.parameter.array.array('d', [0.0]*9)), dtype=np.float64)
    self.linear_acceleration_covariance = np.array(params.optional('linear_acceleration_covariance', rclpy.parameter.array.array('d', [0.0]*9)), dtype=np.float64)

  def get_topics(self):
    return [
      OutputTopic(self.topic, sensor_msgs.msg.Imu, self.qos)
    ]

  def tick(self, workspace):
    return {
      self.topic: [
        (
          sample['time'],
          sensor_msgs.msg.Imu(
            header = msg_helpers.header(sample['time'], self.frame_id),
            orientation = msg_helpers.quaternion(sample['orientation']),
            angular_velocity = msg_helpers.vector3(sample['angular_velocity']),
            linear_acceleration = msg_helpers.vector3(sample['linear_acceleration']),
            orientation_covariance = self.orientation_covariance,
            angular_velocity_covariance = self.angular_velocity_covariance,
            linear_acceleration_covariance = self.linear_acceleration_covariance
          )
        )
        for sample in workspace[self.source]
      ]
    }


@factory('pose', FACTORY_MAP)
class PoseOutput:
  def __init__(self, node, prefix):
    params = ParameterParser(node, prefix)

    self.source = params.expected('source', rclpy.parameter.ParameterType.PARAMETER_STRING)
    self.topic = params.expected('topic', rclpy.parameter.ParameterType.PARAMETER_STRING)
    self.qos = params.optional('qos', None, ParameterType.PARAMETER_STRING)
    self.covariance = np.array(params.optional('covariance', rclpy.parameter.array.array('d', [0.0]*36)), dtype=np.float64)
    self.frame_id = params.optional('frame_id', 'map')

  def get_topics(self):
    return [
      OutputTopic(self.topic, geometry_msgs.msg.PoseWithCovarianceStamped, self.qos)
    ]

  def tick(self, workspace):
    return {
      self.topic: [
        (
          sample['time'],
          geometry_msgs.msg.PoseWithCovarianceStamped(
            header = msg_helpers.header(sample['time'], self.frame_id),
            pose = geometry_msgs.msg.PoseWithCovariance(
              pose = geometry_msgs.msg.Pose(
                position = msg_helpers.point(sample['position']),
                orientation = msg_helpers.quaternion(sample['orientation']),
              ),
              covariance = self.covariance
            )
          )
        )
        for sample in workspace[self.source]
      ]
    }


@factory('gps', FACTORY_MAP)
class GPSOutput:
  def __init__(self, node: rclpy.node.Node, prefix):
    params = ParameterParser(node, prefix)

    self.source = params.expected('source', rclpy.parameter.ParameterType.PARAMETER_STRING)
    self.topic = params.expected('topic', rclpy.parameter.ParameterType.PARAMETER_STRING)
    self.qos = params.optional('qos', None, ParameterType.PARAMETER_STRING)
    self.position_covariance = np.array(params.optional('position_covariance', rclpy.parameter.array.array('d', [0.0]*9)), dtype=np.float64)
    self.frame_id = params.optional('frame_id', 'map')

  def get_topics(self):
    return [
      OutputTopic(self.topic, sensor_msgs.msg.NavSatFix, self.qos)
    ]

  def tick(self, workspace):
    return {
      self.topic: [
        (
          sample['time'],
          sensor_msgs.msg.NavSatFix(
            header = msg_helpers.header(sample['time'], self.frame_id),
            status = sensor_msgs.msg.NavSatStatus(
              status = sensor_msgs.msg.NavSatStatus.STATUS_FIX,
              service = sensor_msgs.msg.NavSatStatus.SERVICE_GPS
            ),
            latitude = sample['latitude'],
            longitude = sample['longitude'],
            altitude = math.nan,
            position_covariance_type = sensor_msgs.msg.NavSatFix.COVARIANCE_TYPE_KNOWN,
            position_covariance = self.position_covariance
          )
        )
        for sample in workspace[self.source]
      ]
    }