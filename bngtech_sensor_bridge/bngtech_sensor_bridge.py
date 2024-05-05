import rclpy.node
import factories
import beamngpy
import beamngpy.sensors
import rclpy.parameter
import rcl_interfaces.msg

class BeamNGTechSensorBridgeNode(rclpy.node.Node):
  EGO_VEHICLE_ID = 'ego_vehicle'
  CLASSIC_SENSOR_FACTORY_MAP = {
    'electrics': beamngpy.sensors.Electrics,
    'state': beamngpy.sensors.State,
    'timer': beamngpy.sensors.Timer
  }

  def __init__(self):
    super().__init__("bngtech_sensor_bridge")

  def __enter__(self):
    self.init()
    return self
  
  def __exit__(self, *_):
    self.deinit()

  def init(self):
    self._init_sim()

    self._init_classic_sensors()
    self.automated_sensors = factories.parse_list(self, 'automated_sensors', factories.automated_sensors.FACTORY_MAP)
    self.outputs = factories.parse_list(self, 'outputs', factories.outputs.FACTORY_MAP)
    self.get_logger().info("Setup complete!")

    self.create_timer(0, self.tick)

  def deinit(self):
    if hasattr(self, 'automated_sensors'):
      for x in self.automated_sensors:
        x.remove()
    if hasattr(self, 'bng'):
      self.bng.close()

  def _init_sim(self):
    self.declare_parameters('settings', [
      ('ip', '10.5.5.10'),
      ('port', 64256),
      ('launch', False),
      ('level', 'smallgrid'),
      ('vehicle.model', 'etk800'),
      ('vehicle.position', rclpy.parameter.array.array('d', [0,0,0])),
      ('vehicle.orientation', rclpy.parameter.array.array('d', [0,0,0,1])),
      ('ai_span', False),
      ('deterministic_hz', -1)
    ])
    params = self.get_parameters_by_prefix('settings')

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

      self.scenario = beamngpy.Scenario(params['level'].value, 'bngtech_bridge')
      self.vehicle = beamngpy.Vehicle(self.EGO_VEHICLE_ID, model=params['vehicle.model'].value)
      self.scenario.add_vehicle(self.vehicle, pos=params['vehicle.position'].value, rot_quat=params['vehicle.orientation'].value, cling=True)
      self.scenario.make(self.bng)

      if params['deterministic_hz'].value > 0:
        self.get_logger().info("Setting deterministic mode.")
        self.bng.settings.set_deterministic(params['deterministic_hz'].value)

      self.get_logger().info("Loading the scenario...")
      self.bng.scenario.load(self.scenario)

      if params['ai_span'].value:
        self.bng.ui.hide_hud()

      self.get_logger().info("Starting scenario.")
      
      self.bng.scenario.start()
      if params['ai_span'].value:
        self.vehicle.ai.set_mode('span')

  def _init_classic_sensors(self):
    for name in set(self.declare_parameter('classic_sensors', descriptor=rcl_interfaces.msg.ParameterDescriptor(type=rclpy.parameter.ParameterType.PARAMETER_STRING_ARRAY)).value):
      if name in self.vehicle.sensors._sensors:
        continue
      self.vehicle.sensors.attach(name, self.CLASSIC_SENSOR_FACTORY_MAP[name]())

  def _poll_automated_sensors(self):
    ans = {}
    for sensor in self.automated_sensors:
      data = sensor.poll()
      if isinstance(data, dict):
        ans[sensor.name] = data.values()
      else:
        ans[sensor.name] = []
    return ans
  
  def _poll_classic_sensors(self):
    self.vehicle.poll_sensors()
    return self.vehicle.sensors.data
  
  def tick(self):
    data = {}
    data.update(self._poll_automated_sensors())
    data.update(self._poll_classic_sensors())

    for output in self.outputs:
      output.tick(data)

if __name__ == '__main__':
  rclpy.init()
  with BeamNGTechSensorBridgeNode() as node:
    rclpy.spin(node)
  rclpy.shutdown()