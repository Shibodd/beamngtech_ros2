import beamngpy
import beamngpy.sensors
from rosbags.typesys import get_typestore
from rosbags.typesys.stores import Stores

import signal
import bag_writer

import sensors.advanced_imu
import sensors.classic_sensors 

from sensors.msg_iface import sensors_to_msg

print("Connecting")
bng = beamngpy.BeamNGpy("192.168.1.2", 64256, quit_on_close=False)

try:
  bng.open(launch=False)
  print("Connected")

  if 'ego_vehicle' in bng.vehicles.get_current():
    vehicle = bng.vehicles.get_current()['ego_vehicle']
    vehicle.connect(bng)
  else:
    scenario = beamngpy.Scenario('west_coast_usa', 'advanced_IMU_demo', description='Spanning the map with an advanced IMU sensor')
    vehicle = beamngpy.Vehicle('ego_vehicle', model='etk800', license='RED', color='Red')

    scenario.add_vehicle(vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795))
    scenario.make(bng)

    bng.settings.set_deterministic(200)

    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    vehicle.ai.set_mode('span')
  
  typestore = get_typestore(Stores.ROS2_HUMBLE)
  vehicle.sensors.attach('electrics', beamngpy.sensors.Electrics())
  vehicle.sensors.attach('timer', beamngpy.sensors.Timer())
  
  
  imu = sensors.advanced_imu.AdvancedIMUSensor(typestore, 'imu',
    beamngpy.sensors.AdvancedIMU('accel1', bng, vehicle, physics_update_time=0.003)
  )
  # gps = beamngpy.sensors.GPS('gps1', bng, vehicle, physics_update_time=0.2)
  classic = sensors.classic_sensors.ClassicSensors(typestore, vehicle, {
    'pose': ('state', sensors.classic_sensors.odometry_pose, 0.1),
    'pose_gt': ('state', sensors.classic_sensors.odometry_pose, 0),
    'tf': ('state', sensors.classic_sensors.vehicle_tf, 0.1),
    'wheelspeed': ('electrics', sensors.classic_sensors.twist_wheelspeed, 0.01),
  }, 'timer')

  topics = {
    'imu': ('/bng/imu', ''),
    'imu_pose': ('/bng/imu/pose', ''),
    'pose': ('/bng/pose', ''),
    'pose_gt': ('/bng/pose_gt', ''),
    'tf': ('/tf', ''),
    'wheelspeed': ('/bng/wheelspeed', '')
  }

  print("Running - Press CTRL+C to stop.")
  
  should_quit = False
  writer_err = None
  def handle_sigint(_, __):
    global should_quit
    should_quit = True
    print("Stop requested")

  def error_handler(err):
    global writer_err, should_quit
    writer_err = err
    should_quit = True

  signal.signal(signal.SIGINT, handle_sigint)

  with bag_writer.BagWriter("output.bag", topics, typestore, error_handler) as writer:
    while not should_quit:
      classic_data = classic.poll_data()
      imu_data = imu.poll_data()

      writer.add_msgs(sensors_to_msg(typestore, classic_data))
      writer.add_msgs(sensors_to_msg(typestore, imu_data))

    if writer_err:
      raise writer_err
finally:
  imu.IMU.remove()
  bng.close()