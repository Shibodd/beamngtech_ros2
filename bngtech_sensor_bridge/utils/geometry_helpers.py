import scipy.spatial.transform
import numpy as np

def matrix_from_axes(x, y, z):
  rot = np.hstack((
    np.array(x, dtype=np.float64).reshape(3, 1),
    np.array(y, dtype=np.float64).reshape(3, 1),
    np.array(z, dtype=np.float64).reshape(3, 1)
  ))
  return rot

def quat_from_matrix(mat):
  return scipy.spatial.transform.Rotation.from_matrix(mat).as_quat()

def quat_from_axes(x, y, z):
  return quat_from_matrix(matrix_from_axes(x, y, z))

def quat_from_fwd_up(fwd, up):
  fwd = np.array(fwd, dtype=np.float64).reshape(3,)
  up = np.array(up, dtype=np.float64).reshape(3,)
  return quat_from_axes(fwd, np.cross(up, fwd), up)