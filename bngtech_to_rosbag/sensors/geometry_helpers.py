import scipy.spatial.transform
import numpy as np

def quat_from_axes(x, y, z):
  rot = np.hstack((
    np.array(x).reshape(3, 1),
    np.array(y).reshape(3, 1),
    np.array(z).reshape(3, 1)
  ))
  return scipy.spatial.transform.Rotation.from_matrix(rot).as_quat()

def quat_from_fwd_up(fwd, up):
  return quat_from_axes(fwd, up, np.cross(fwd, up))