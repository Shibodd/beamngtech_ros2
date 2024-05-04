import re
import numpy as np

import geometry_helpers
import msg_helpers

import geometry_msgs.msg
import sensor_msgs.msg
import tf2_msgs.msg

import math

IMU_REGEX = re.compile("^imu(\d+)\.(data|pose)$")
GPS_REGEX = re.compile("^gps(\d+)")

def get_message_parser(k):
  if m := IMU_REGEX.match(k):
    i, f = m.groups()
    i = int(i)

    match f:
      case 'data':
        return lambda data: parse_imu(i, data)
      case 'pose':
        return lambda data: parse_imu_pose(i, data)
    assert False

  if m := GPS_REGEX.match(k):
    i, = m.groups()
    return lambda data: parse_gps(i, data)

  match k:
    case 'pose':
      return parse_pose
    case 'tf':
      return parse_tf
    case 'wheelspeed':
      return parse_wheelspeed
  assert(False)


def parse_imu_pose(i, data):
  def parse_sample(sample):
    orientation = geometry_helpers.quat_from_fwd_up(sample['dirX'], sample['dirZ'])
    return geometry_msgs.msg.PoseStamped(
      header = msg_helpers.header(sample['time'], 'map'),
      pose = geometry_msgs.msg.Pose(
        position = msg_helpers.point(sample['pos']),
        orientation = msg_helpers.quaternion(orientation),
      )
    )
  return [parse_sample(sample) for sample in data[f'imu{i}']]

def parse_imu(i, data):
  def parse_sample(sample):
    COVARIANCE = np.zeros((9,), dtype=np.float64)

    orientation = geometry_helpers.quat_from_axes(sample['dirX'], sample['dirY'], sample['dirZ'])

    return sensor_msgs.msg.Imu(
      header = msg_helpers.header(sample['time'], 'imu_link'),
      orientation = msg_helpers.quaternion(orientation),
      angular_velocity = msg_helpers.vector3(sample['angVel']),
      linear_acceleration = msg_helpers.vector3(sample['accSmooth']),
      
      orientation_covariance = COVARIANCE,
      angular_velocity_covariance = COVARIANCE,
      linear_acceleration_covariance = COVARIANCE
    )
  return [parse_sample(sample) for sample in data[f'imu{i}']]

def parse_pose(data):
  state = data['state']
  position = np.array(state['pos']).reshape((3,))
  orientation = geometry_helpers.quat_from_fwd_up(state['dir'], state['up'])

  return [geometry_msgs.msg.PoseStamped(
    header = msg_helpers.header(data['timer']['time'], 'map'),
    pose = geometry_msgs.msg.Pose(
      position = msg_helpers.point(position),
      orientation = msg_helpers.quaternion(orientation)
    )
  )]

def parse_tf(data):
  state = data['state']
  position = np.array(state['pos'], dtype=np.float64).reshape((3,))
  orientation = geometry_helpers.quat_from_fwd_up(state['dir'], state['up'])

  return [
    tf2_msgs.msg.TFMessage(
      transforms = [
        geometry_msgs.msg.TransformStamped(
          header = msg_helpers.header(data['timer']['time'], 'map'),
          child_frame_id = 'imu_link',
          transform = geometry_msgs.msg.Transform(
            translation = msg_helpers.vector3(position),
            rotation = msg_helpers.quaternion(orientation)
          )
        )
      ]
    )
  ]

def parse_wheelspeed(data):
  electrics = data['electrics']
  speed = electrics['wheelspeed'] * (1 if electrics['gear'] != 'R' else -1)

  return [geometry_msgs.msg.TwistStamped(
    header = msg_helpers.header(data['timer']['time'], 'imu_link'),
    twist = geometry_msgs.msg.Twist(
      linear = msg_helpers.vector3(np.array([speed, 0, 0], dtype=np.float64)),
      angular = msg_helpers.vector3(np.zeros((3,), dtype=np.float64))
    )
  )]

def parse_gps(i, data):
  def parse_sample(sample):
    return sensor_msgs.msg.NavSatFix(
      header = msg_helpers.header(sample['time'], 'map'),
      status = sensor_msgs.msg.NavSatStatus(
        status = sensor_msgs.msg.NavSatStatus.STATUS_FIX,
        service = sensor_msgs.msg.NavSatStatus.SERVICE_GPS
      ),
      latitude = sample['lat'],
      longitude = sample['lon'],
      altitude = math.nan,
      position_covariance_type = sensor_msgs.msg.NavSatFix.COVARIANCE_TYPE_KNOWN,
      position_covariance = np.zeros((9,), dtype=np.float64)
    )
  return [parse_sample(sample) for sample in data[f'gps{i}']]