from .factories import factory
from .param_parser import ParameterParser, ParameterType

import rclpy.node
import rclpy.parameter
import numpy as np
import math

import geometry_helpers
import geometry_msgs.msg
import tf2_msgs.msg
import msg_helpers
import sensor_msgs.msg

FACTORY_MAP = {}


@factory('car_pose', FACTORY_MAP)
class CarPoseOutput:
  def __init__(self, node: rclpy.node.Node, prefix):
    params = ParameterParser(node, prefix)

    self.pub = node.create_publisher(
      geometry_msgs.msg.PoseWithCovarianceStamped,
      params.expected('topic', ParameterType.PARAMETER_STRING),
      10
    )
    self.frame_id = params.optional('frame_id', 'map')
    self.covariance = np.array(params.optional('covariance', rclpy.parameter.array.array('d', [0.0]*36)), dtype=np.float64)

  def tick(self, data):
    state = data['state']
    position = np.array(state['pos']).reshape((3,))
    orientation = geometry_helpers.quat_from_fwd_up(state['dir'], state['up'])

    self.pub.publish(
      geometry_msgs.msg.PoseWithCovarianceStamped(
        header = msg_helpers.header(data['timer']['time'], self.frame_id),
        pose = geometry_msgs.msg.PoseWithCovariance(
          pose = geometry_msgs.msg.Pose(
            position = msg_helpers.point(position),
            orientation = msg_helpers.quaternion(orientation)
          ),
          covariance = self.covariance
        )
      )
    )


@factory('car_tf', FACTORY_MAP)
class CarTFOutput:
  def __init__(self, node: rclpy.node.Node, prefix):
    params = ParameterParser(node, prefix)

    self.pub = node.create_publisher(
      tf2_msgs.msg.TFMessage,
      params.expected('topic', ParameterType.PARAMETER_STRING),
      10
    )
    self.frame_id = params.optional('frame_id', 'map')
    self.child_frame_id = params.optional('child_frame_id', 'imu_link')

  def tick(self, data):
    state = data['state']
    position = np.array(state['pos'], dtype=np.float64).reshape((3,))
    orientation = geometry_helpers.quat_from_fwd_up(state['dir'], state['up'])

    self.pub.publish(
      tf2_msgs.msg.TFMessage(
        transforms = [
          geometry_msgs.msg.TransformStamped(
            header = msg_helpers.header(data['timer']['time'], self.frame_id),
            child_frame_id = self.child_frame_id,
            transform = geometry_msgs.msg.Transform(
              translation = msg_helpers.vector3(position),
              rotation = msg_helpers.quaternion(orientation)
            )
          )
        ]
      )
    )
  

@factory('wheelspeed', FACTORY_MAP)
class WheelspeedOutput:
  def __init__(self, node: rclpy.node.Node, prefix):
    params = ParameterParser(node, prefix)

    self.pub = node.create_publisher(
      geometry_msgs.msg.TwistWithCovarianceStamped,
      params.expected('topic', ParameterType.PARAMETER_STRING),
      10
    )
    self.frame_id = params.optional('frame_id', 'map')
    self.variance = params.optional('variance', 0.0)

  def tick(self, data):
    electrics = data['electrics']
    speed = electrics['wheelspeed'] * (1 if electrics['gear'] != 'R' else -1)

    self.pub.publish(
      geometry_msgs.msg.TwistWithCovarianceStamped(
        header = msg_helpers.header(data['timer']['time'], self.frame_id),
        twist = geometry_msgs.msg.TwistWithCovariance(
          twist = geometry_msgs.msg.Twist(
            linear = msg_helpers.vector3(np.array([speed, 0, 0], dtype=np.float64)),
            angular = msg_helpers.vector3(np.zeros((3,), dtype=np.float64))
          ),
          covariance = np.array([self.variance] + [0.0]*35, dtype=np.float64)
        )
      )
    )


@factory('imu_data', FACTORY_MAP)
class IMUDataOutput:
  def __init__(self, node: rclpy.node.Node, prefix):
    params = ParameterParser(node, prefix)
    
    self.source = params.expected('source', rclpy.parameter.ParameterType.PARAMETER_STRING)
    self.pub = node.create_publisher(
      sensor_msgs.msg.Imu,
      params.expected('topic', rclpy.parameter.ParameterType.PARAMETER_STRING),
      10
    )

    self.frame_id = params.optional('frame_id', 'imu_link')
    self.orientation_covariance = np.array(params.optional('orientation_covariance', rclpy.parameter.array.array('d', [0.0]*9)), dtype=np.float64)
    self.angular_velocity_covariance = np.array(params.optional('angular_velocity_covariance', rclpy.parameter.array.array('d', [0.0]*9)), dtype=np.float64)
    self.linear_acceleration_covariance = np.array(params.optional('linear_acceleration_covariance', rclpy.parameter.array.array('d', [0.0]*9)), dtype=np.float64)

  def tick(self, data):
    for sample in data[self.source]:
      orientation = geometry_helpers.quat_from_axes(sample['dirX'], sample['dirY'], sample['dirZ'])

      self.pub.publish(
        sensor_msgs.msg.Imu(
          header = msg_helpers.header(sample['time'], self.frame_id),
          orientation = msg_helpers.quaternion(orientation),
          angular_velocity = msg_helpers.vector3(sample['angVel']),
          linear_acceleration = msg_helpers.vector3(sample['accSmooth']),
          orientation_covariance = self.orientation_covariance,
          angular_velocity_covariance = self.angular_velocity_covariance,
          linear_acceleration_covariance = self.linear_acceleration_covariance
        )
      )


@factory('imu_pose', FACTORY_MAP)
class IMUPoseOutput:
  def __init__(self, node, prefix):
    params = ParameterParser(node, prefix)

    self.source = params.expected('source', rclpy.parameter.ParameterType.PARAMETER_STRING)
    self.pub = node.create_publisher(
      geometry_msgs.msg.PoseWithCovarianceStamped,
      params.expected('topic', rclpy.parameter.ParameterType.PARAMETER_STRING),
      10
    )
    self.covariance = np.array(params.optional('covariance', rclpy.parameter.array.array('d', [0.0]*36)), dtype=np.float64)
    self.frame_id = params.optional('frame_id', 'map')

  def tick(self, data):
    for sample in data[self.source]:
      orientation = geometry_helpers.quat_from_fwd_up(sample['dirX'], sample['dirZ'])

      self.pub.publish(
        geometry_msgs.msg.PoseWithCovarianceStamped(
          header = msg_helpers.header(sample['time'], self.frame_id),
          pose = geometry_msgs.msg.PoseWithCovariance(
            pose = geometry_msgs.msg.Pose(
              position = msg_helpers.point(sample['pos']),
              orientation = msg_helpers.quaternion(orientation),
            ),
            covariance = self.covariance
          )
        )
      )


@factory('gps', FACTORY_MAP)
class GPSOutput:
  def __init__(self, node: rclpy.node.Node, prefix):
    params = ParameterParser(node, prefix)

    self.source = params.expected('source', rclpy.parameter.ParameterType.PARAMETER_STRING)
    self.pub = node.create_publisher(
      sensor_msgs.msg.NavSatFix,
      params.expected('topic', rclpy.parameter.ParameterType.PARAMETER_STRING),
      10
    )
    self.position_covariance = np.array(params.optional('position_covariance', rclpy.parameter.array.array('d', [0.0]*9)), dtype=np.float64)
    self.frame_id = params.optional('frame_id', 'map')


  def tick(self, data):
    for sample in data[self.source]:
      self.pub.publish(
        sensor_msgs.msg.NavSatFix(
          header = msg_helpers.header(sample['time'], self.frame_id),
          status = sensor_msgs.msg.NavSatStatus(
            status = sensor_msgs.msg.NavSatStatus.STATUS_FIX,
            service = sensor_msgs.msg.NavSatStatus.SERVICE_GPS
          ),
          latitude = sample['lat'],
          longitude = sample['lon'],
          altitude = math.nan,
          position_covariance_type = sensor_msgs.msg.NavSatFix.COVARIANCE_TYPE_KNOWN,
          position_covariance = self.position_covariance
        )
      )