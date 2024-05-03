import beamngpy
import beamngpy.sensors
from rosbags.typesys import get_typestore
from rosbags.typesys.stores import Stores

import signal
import bag_writer

import sensors.advanced_imu
import sensors.classic_sensors 
import sensors.msg_iface as msg_iface
import transforms

from qos import TRANSIENT_LOCAL_QOS


print("Connecting")
TRANSFORM = True
MANUAL = True
bng = beamngpy.BeamNGpy("10.5.5.10", 64256, quit_on_close=False)

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

    if not MANUAL:
      bng.settings.set_deterministic(200)

    bng.scenario.load(scenario)

    if not MANUAL:
      bng.ui.hide_hud()
    bng.scenario.start()

    if not MANUAL:
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
    'tf_gt': ('state', sensors.classic_sensors.vehicle_tf, 0),
    'wheelspeed': ('electrics', sensors.classic_sensors.twist_wheelspeed, 0.01),
  }, 'timer')

  topics = {
    'pose_gt': ('/bng/gt/pose', ''),
    'tf_gt': ('/bng/gt/tf', ''),
    'pose': ('/bng/pose', ''),
    'wheelspeed': ('/bng/wheelspeed', ''),
    'imu': ('/bng/imu', ''),
    'tf_static': ('/tf_static', TRANSIENT_LOCAL_QOS)
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

  ORIENTATION_TF = transforms.BngOrientationTransform()
  track_tf = None

  start_t = None

  with bag_writer.BagWriter("output.bag", topics, typestore, error_handler) as writer:
    while not should_quit:
      data = {}
      data.update(classic.poll_data())
      data.update(imu.poll_data())

      if start_t is None:
        time_k = next((k for k,v in data.items() if len(v) > 0), None)
        if time_k != None:
          start_t = data[time_k][-1].time
      
      for v in data.values():
        for x in v:
          x.time = x.time - start_t

      for k in ['pose_gt', 'tf_gt', 'pose', 'imu']:
        for v in data[k]:
          v.apply_tf(ORIENTATION_TF)

      for v in data['imu']:
        v.linear_acceleration.data[1] = -v.linear_acceleration.data[1]

      if TRANSFORM:
        if track_tf is None and len(data['pose_gt']) > 0:
          pose = data['pose_gt'][-1]
          pose: msg_iface.PoseData
          track_tf = transforms.MapToTrackTransform(pose.position.data, pose.orientation.data)
          data['tf_static'] = data.get('tf_static', []) + [track_tf.tf_data]

        if track_tf is not None:
          for k in ['pose_gt', 'tf_gt', 'pose']:
            for v in data[k]:
              v.apply_tf(track_tf)

      for v in data['pose']:
        v.position.add_variance(0.1)
        v.orientation.add_variance(0.1)
      
      for v in data['imu']:
        v.orientation.add_variance(0.1)
        v.angular_velocity.add_variance(0.1)
        v.linear_acceleration.add_variance(0.1)
        
      if not TRANSFORM or track_tf is not None:
        writer.add_data(data)
    if writer_err:
      raise writer_err
finally:
  imu.IMU.remove()
  bng.close()