import numpy as np

def concat_indip_covariances(*cov):
  if len(cov) == 0:
    return np.array([])
  if len(cov) == 1:
    return cov[0]
  
  A = cov[0]
  B = concat_indip_covariances(*cov[1:])

  TR = np.zeros((A.shape[0], B.shape[1]))
  BL = np.zeros((B.shape[0], A.shape[1]))

  return np.block([[A, TR],
                   [BL, B]])