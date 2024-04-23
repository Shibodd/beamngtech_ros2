import beamngpy
import beamngpy.sensors
import json
from rosbags.typesys import get_typestore
from rosbags.typesys.stores import Stores

from sensors.advanced_imu import AdvancedIMUSensor
from sensors.classic_sensors import ClassicSensors

print("Connecting")
bng = beamngpy.BeamNGpy("192.168.1.2", 64256, quit_on_close=False)

SETUP = False

try:
  bng.open(launch=False)
  print("Connected")
  if SETUP:
    scenario = beamngpy.Scenario('west_coast_usa', 'advanced_IMU_demo', description='Spanning the map with an advanced IMU sensor')
    vehicle = beamngpy.Vehicle('ego_vehicle', model='etk800', license='RED', color='Red')

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)

    bng.settings.set_deterministic(60) # Set simulator to 60hz temporal resolution

    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    vehicle.ai.set_mode('span')
  else:
    vehicle = bng.vehicles.get_current()['ego_vehicle']
    vehicle.connect(bng)

  imu_sensor = AdvancedIMUSensor(get_typestore(Stores.ROS2_HUMBLE), beamngpy.sensors.AdvancedIMU('accel1', bng, vehicle, gfx_update_time=0.005))
  vehicle.sensors.attach('electrics', beamngpy.sensors.Electrics())
  vehicle.sensors.attach('timer', beamngpy.sensors.Timer())
  classic_sensors = ClassicSensors(vehicle, {}, 'timer')  

  print("Running - Press CTRL+C to stop.")
  import pprint

  try:
    while True:
      imu_sensor.poll_msgs()
      classic_sensors.poll_msgs()
      # pprint.pprint(IMU.poll_msgs())
  except KeyboardInterrupt:
    print("Stop requested - stopping...")
finally:
  bng.close()