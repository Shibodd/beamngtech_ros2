import rclpy.node
import beamngpy
import beamngpy.sensors
import rclpy.parameter
import geometry_msgs.msg
import msg_helpers
import tf2_msgs.msg
import sensor_msgs.msg

import re
import geometry_helpers
import numpy as np
import math

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

  return [geometry_msgs.msg.TwistStamped(
    header = msg_helpers.header(data['timer']['time'], 'imu_link'),
    twist = geometry_msgs.msg.Twist(
      linear = msg_helpers.vector3(np.array([electrics['wheelspeed'], 0, 0], dtype=np.float64)),
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

class BeamNGTechSensorBridgeNode(rclpy.node.Node):
  EGO_VEHICLE_ID = 'ego_vehicle'

  def __init__(self):
    super().__init__("bngtech_bridge")

    self.initialized = False

    self.declare_parameters('bngtech.sim', [
      ('ip', '10.5.5.10'),
      ('port', 64256),
      ('launch', False),
      ('level', 'west_coast_usa'),
      ('vehicle.model', 'etk800'),
      ('vehicle.position', rclpy.parameter.array.array('d', [-717.121, 101, 118.675])),
      ('vehicle.orientation', rclpy.parameter.array.array('d', [0, 0, 0.3826834, 0.9238795])),
      ('ai_span', False),
      ('deterministic_hz', -1),
      ('publish_sim_time', True)
    ])

    self.declare_parameters('bngtech.sensors', [
      ('imu0.topic', '/bng/imu'),
      ('imu0.update_time', 0.003),
      ('pose.topic', '/bng/pose'),
      ('tf.topic', '/tf'),
      ('wheelspeed.topic', '/bng/wheelspeed'),
      ('gps0.topic', '/bng/gps')
    ])

  def init(self):
    self.initialized = True
    self._init_sim()
    self._init_sensors()
    # self.time_pub = self.create_publisher(rosgraph_msgs.msg.Clock, '/clock', 10)
    self.get_logger().info("Initialized!")

  def _init_sim(self):
    params = self.get_parameters_by_prefix('bngtech.sim')

    self.get_logger().info("Connecting to BeamNG...")
    self.bng = beamngpy.BeamNGpy(params['ip'].value, params['port'].value, quit_on_close=False)
    self.bng.open(launch=params['launch'].value)
    self.get_logger().info("Connected.")

    if self.EGO_VEHICLE_ID in self.bng.vehicles.get_current():
      self.get_logger().info("ego_vehicle found - reusing the current scenario.")
      self.vehicle = self.bng.vehicles.get_current()[self.EGO_VEHICLE_ID]
      self.vehicle.connect(self.bng)
    else:
      self.get_logger().info("Setting up the scenario...")

      # self.scenario = beamngpy.Scenario(params['level'].value, 'bngtech_bridge')
      self.scenario = beamngpy.Scenario('smallgrid', 'bngtech_bridge')
      self.vehicle = beamngpy.Vehicle(self.EGO_VEHICLE_ID, model=params['vehicle.model'].value)
      #self.scenario.add_vehicle(self.vehicle, pos=params['vehicle.position'].value, rot_quat=params['vehicle.orientation'].value)
      self.scenario.add_vehicle(self.vehicle)
      self.scenario.make(self.bng)

      if params['deterministic_hz'].value > 0:
        self.get_logger().info("Setting deterministic mode.")
        self.bng.settings.set_deterministic(params['deterministic_hz'].value)

      self.get_logger().info("Loading the scenario...")
      self.bng.scenario.load(self.scenario)

      if params['ai_span'].value:
        self.bng.ui.hide_hud()

      self.bng.scenario.start()
      if params['ai_span'].value:
        self.vehicle.ai.set_mode('span')

  def _init_sensors(self):
    params = self.get_parameters_by_prefix('bngtech.sensors')

    self.get_logger().info("Adding sensors.")
    self.vehicle.sensors.attach('timer', beamngpy.sensors.Timer())
    self.vehicle.sensors.attach('electrics', beamngpy.sensors.Electrics())
    self.automated_sensors = {
      'imu0': beamngpy.sensors.AdvancedIMU('imu0', self.bng, self.vehicle, physics_update_time=params['imu0.update_time'].value, is_using_gravity=True, dir=(0,-1,0), up=(1,0,0)),
      'gps0': beamngpy.sensors.GPS('gps0', self.bng, self.vehicle)
    }

    self.output_publishers = {
      'imu0.data': self.create_publisher(sensor_msgs.msg.Imu, f"{params['imu0.topic'].value}/data", 10),
      'imu0.pose': self.create_publisher(geometry_msgs.msg.PoseStamped, f"{params['imu0.topic'].value}/pose", 10),
      'wheelspeed': self.create_publisher(geometry_msgs.msg.TwistStamped, params['wheelspeed.topic'].value, 10),
      'pose': self.create_publisher(geometry_msgs.msg.PoseStamped, params['pose.topic'].value, 10),
      'tf': self.create_publisher(tf2_msgs.msg.TFMessage, params['tf.topic'].value, 10),
      'gps0': self.create_publisher(sensor_msgs.msg.NavSatFix, params['gps0.topic'].value, 10)
    }

  def _poll_automated_sensors(self):
    ans = {}
    for k, v in self.automated_sensors.items():
      data = v.poll()
      if isinstance(data, dict):
        ans[k] = data.values()
      else:
        ans[k] = []
    return ans
  
  def _poll_classic_sensors(self):
    self.vehicle.poll_sensors()
    return self.vehicle.sensors.data
  
  def tick(self):
    data = {}
    data.update(self._poll_automated_sensors())
    data.update(self._poll_classic_sensors())

    for output_name, pub in self.output_publishers.items():
      parser = get_message_parser(output_name)
      for msg in parser(data):
        pub.publish(msg)

  def deinit(self):
    if not self.initialized:
      return
    
    for x in self.automated_sensors.values():
      x.remove()
    self.bng.close()


import signal

should_quit = False
def handle_sigint(_, __):
  global should_quit
  should_quit = True

signal.signal(signal.SIGINT, handle_sigint)

try:
  rclpy.init()
  node = BeamNGTechSensorBridgeNode()
  node.init()
  while not should_quit:
    node.tick()
finally:
  node.deinit()
  if node.context.ok():
    rclpy.shutdown()