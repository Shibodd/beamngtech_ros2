from scipy.spatial.transform import Rotation
import numpy as np

class BngOrientationTransform:
  # BeamNG reports orientations with a different convention
  # BeamNG: X forward, Y up,   Z right
  #    ROS: X forward, Y left, Z up
  ROT = Rotation.from_matrix(
    np.array([[1, 0, 0],
              [0, 0, 1],
              [0, -1, 0]])
  )

  def transform_vector(self, v):
    return self.ROT.apply(v)

  def transform_orientation(self, quat):
    return (Rotation(quat) * self.ROT).as_quat()
  
  def transform_position(self, p):
    return p