import rclpy.node
import beamngpy
import beamngpy.sensors
import rclpy.parameter
import geometry_msgs.msg

import tf2_msgs.msg
import sensor_msgs.msg

import parsers

class BeamNGTechSensorBridgeNode(rclpy.node.Node):
  EGO_VEHICLE_ID = 'ego_vehicle'

  def __init__(self):
    super().__init__("bngtech_bridge")

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
      parser = parsers.get_message_parser(output_name)
      for msg in parser(data):
        pub.publish(msg)

  def deinit(self):
    for x in getattr(self, 'automated_sensors', {}).values():
      x.remove()
    
    if hasattr(self, 'bng'):
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