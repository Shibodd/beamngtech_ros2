import rclpy.node
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Transform, Vector3, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import Imu

import tf2_ros

import numpy as np
from scipy.spatial.transform import Rotation

def np_from_msg(msg):
  ans = np.ndarray((4,), np.float64)
  for i, k in enumerate(['x', 'y', 'z', 'w']):
    if not hasattr(msg, k):
      return ans[:i] 
    ans[i] = getattr(msg, k)
  return ans

class BngToLocalTransform:
  # BeamNG reports orientations with a different convention
  # BeamNG: X forward, Y up,   Z right
  #    ROS: X forward, Y left, Z up
  ORIENTATION_BASE_TF = Rotation.from_matrix(
    np.array([[1, 0, 0],
              [0, 0, 1],
              [0, -1, 0]])
  )
  
  def __init__(self, start_position, start_orientation):
    self.origin = start_position
    self.rotation = (Rotation(start_orientation) * self.ORIENTATION_BASE_TF).inv()

  def transform_vector(self, v):
    return self.rotation.apply(v)

  def transform_position(self, p):
    return self.transform_vector(p - self.origin)

  def transform_orientation(self, quat):
    rot = Rotation(quat) * self.ORIENTATION_BASE_TF
    return (self.rotation * rot).as_quat()

class MyNode(rclpy.node.Node):
  def __init__(self):
    super().__init__("tf_test")
    self.sub = self.create_subscription(PoseWithCovarianceStamped, "/bng/gt/pose", self.gt_cb, 10)
    #self.sub = self.create_subscription(Imu, "/bng/imu", self.imu_cb, 10)
    self.pub = tf2_ros.TransformBroadcaster(self)
    self.tf = None

  def pub_tf(self, translation, rotation):
    self.pub.sendTransform(TransformStamped(
      header = Header(
        stamp = self.get_clock().now().to_msg(),
        frame_id = 'track'
      ),
      child_frame_id = 'imu_link',
      transform = Transform(
        translation = Vector3(x=translation[0], y=translation[1], z=translation[2]),
        rotation = Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])
      )
    ))

  def imu_cb(self, msg: Imu):
    orientation = np_from_msg(msg.orientation)

    if self.tf is None:
      self.tf = BngToLocalTransform(np.array([0,0,0]), orientation)

    self.pub_tf(
      np.array([0,0,0], dtype=np.float64),
      self.tf.transform_orientation(orientation)
    )

  def gt_cb(self, msg: PoseWithCovarianceStamped):
    position = np_from_msg(msg.pose.pose.position)
    orientation = np_from_msg(msg.pose.pose.orientation)
    
    if self.tf is None:
      self.tf = BngToLocalTransform(position, orientation)

    self.pub_tf(self.tf.transform_position(position), self.tf.transform_orientation(orientation))

    
rclpy.init()
rclpy.spin(MyNode())
rclpy.shutdown()