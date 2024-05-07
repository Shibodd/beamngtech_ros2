import rclpy.node
import factories
import beamngpy
import beamngpy.sensors
import rclpy.parameter
import itertools
import utils.param_parser as param_parser
import utils.dict_extend
from utils.workspace import Workspace
import writers

class BeamNGTechSensorBridgeNode(rclpy.node.Node):
  EGO_VEHICLE_ID = 'ego_vehicle'

  def __init__(self):
    super().__init__("bngtech_sensor_bridge")

  def __enter__(self):
    self.init()
    return self
  
  def __exit__(self, *_):
    self.deinit()

  def init(self):
    self._init_sim()

    self.classic_sensors = factories.parse_simple_list(self, 'classic_sensors', factories.classic_sensors.FACTORY_MAP)
    self.automated_sensors = factories.parse_list(self, 'automated_sensors', factories.automated_sensors.FACTORY_MAP)
    self.transformers = factories.parse_list(self, 'transformers', factories.transformers.FACTORY_MAP)
    self.outputs = factories.parse_list(self, 'outputs', factories.outputs.FACTORY_MAP)
    self.writers = [ writers.MessagePublisher(self) ]

    self.get_logger().info("Setup complete!")

    self.create_timer(0, self.tick)

  def deinit(self):
    if hasattr(self, 'automated_sensors'):
      for x in self.automated_sensors:
        x.remove()
    if hasattr(self, 'bng'):
      self.bng.close()

  def _init_sim(self):
    params = param_parser.ParameterParser(self, 'settings')

    self.get_logger().info("Connecting to BeamNG...")
    self.bng = beamngpy.BeamNGpy(
      params.expected('ip', param_parser.ParameterType.PARAMETER_STRING),
      params.expected('port', param_parser.ParameterType.PARAMETER_INTEGER),
      quit_on_close=False
    )
    self.bng.open(launch=params.optional('launch', False))
    self.get_logger().info("Connected.")

    if self.EGO_VEHICLE_ID in self.bng.vehicles.get_current():
      self.get_logger().info("ego_vehicle found - reusing the current scenario.")
      self.vehicle = self.bng.vehicles.get_current()[self.EGO_VEHICLE_ID]
      self.vehicle.connect(self.bng)
    else:
      self.get_logger().info("Setting up the scenario...")

      self.scenario = beamngpy.Scenario(params.optional('level', 'smallgrid'), 'bngtech_bridge')
      self.vehicle = beamngpy.Vehicle(self.EGO_VEHICLE_ID, model=params.optional('vehicle.model', 'etk800'))
      self.scenario.add_vehicle(self.vehicle,
        pos=params.optional('vehicle.position', rclpy.parameter.array.array('d', [0,0,0])),
        rot_quat=params.optional('vehicle.rotation', rclpy.parameter.array.array('d', [0,0,0, 1])),
        cling=True
      )
      self.scenario.make(self.bng)

      if det_hz := params.optional('deterministic_hz', -1.0) > 0:
        self.get_logger().info("Setting deterministic mode.")
        self.bng.settings.set_deterministic(det_hz)

      self.get_logger().info("Loading the scenario...")
      self.bng.scenario.load(self.scenario)

      ai_span = params.optional('ai_span', False)
      if ai_span:
        self.bng.ui.hide_hud()

      self.get_logger().info("Starting scenario.")
      
      self.bng.scenario.start()
      if ai_span:
        self.vehicle.ai.set_mode('span')
  
  def tick(self):
    workspace = Workspace()

    self.vehicle.sensors.poll()
    for sensor in itertools.chain(self.classic_sensors, self.automated_sensors):
      sensor.poll(workspace)

    for transformer in self.transformers:
      transformer.tick(workspace)

    msgs = {}
    for output in self.outputs:
      utils.dict_extend.dict_extend(msgs, output.tick(workspace))

    for writer in self.writers:
      writer.write(msgs)


if __name__ == '__main__':
  rclpy.init()
  with BeamNGTechSensorBridgeNode() as node:
    rclpy.spin(node)
  rclpy.shutdown()