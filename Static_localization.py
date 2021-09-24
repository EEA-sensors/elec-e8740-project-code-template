#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 13 14:14:15 2019

@author: muhammad
"""

#%%
%matplotlib auto
import numpy as np
import matplotlib.pyplot as plt
import sensor_fusion as sf
import robot_n_measurement_functions as rnmf
import pathlib
import seaborn as sns
import matplotlib.patches as mpatches
from scipy.linalg import expm
import lsqSolve as lsqS
sns.set()
#%%
parent_path = pathlib.Path.home()
parent_path = parent_path/'Dropbox/09. Aalto Postdoc/DiddyBorg_experiment'
Camera = sf.Sensor('Camera',sf.CAMERA_COLUMNS,meas_record_file=parent_path/'test-run-camera.csv',is_linear=False,start_index=0)
#%%

x_true = np.array([17,60,0])
x_init = x_true + np.random.randn(3)

R_one_diag = np.array([2,10])

# params = {'x_sensors':np.zeros((1,2))}
I_max=100
gamma=1
params_LSQ = {'x_sensors':None,
        'R':None,
        'LR':None,#cholesky factorization of a matrix (chol(a) in matlab returns an upper triangular matrix, but linalg.cholesky(a) returns a lower triangular matrix)
        'Rinv':None,
        'gamma':gamma,
        'I_max':I_max,
        'Line_search':False,
        'Line_search_n_points':10,
        'Jwls':lsqS.Jwls
        }

#%%
Camera.reset_sampling_index()
for i in range(0):
    Camera.get_measurement()
#%%
y_raw = Camera.get_measurement()

weight = y_raw[:,3]
height = y_raw[:,4]
c_x = y_raw[:,1]

dist = rnmf.QRCODE_SIDE_LENGTH*rnmf.PERCEIVED_FOCAL_LENGTH/height
direct = np.arctan2(c_x,rnmf.PERCEIVED_FOCAL_LENGTH) 
angle_qr = np.arccos(np.minimum(weight,height)/height)

corrected_dist = dist/np.cos(direct) + 0.5*rnmf.QRCODE_SIDE_LENGTH*np.sin(angle_qr)
y_raw[:,5] = corrected_dist#dist/np.cos(direct)
n_qr_codes = y_raw.shape[0]
y = y_raw[:,5:].flatten()
#y = y_raw[:,-2]
qr_pos = rnmf.QRCODE_LOCATIONS[y_raw[:,0].astype('int'),1:]
params_LSQ['x_sensors'] = qr_pos


R = np.diag(np.kron(np.ones(n_qr_codes),R_one_diag))
params_LSQ['R'] = R
params_LSQ['LR'] = np.linalg.cholesky(R).T
params_LSQ['Rinv'] = np.diag(1/np.diag(R))
xhat_history_GN, J_history_GN = lsqS.lsqsolve(y,rnmf.h_cam,rnmf.H_cam,x_init,params_LSQ,method='gauss-newton')

#%%
# plt.figure()
#plt.plot(x[:,0],x[:,1],'-ok',linewidth=0.5,markersize=2)
fig, ax = plt.subplots(1,3)
ax[0].plot(xhat_history_GN[0,:])
ax[1].plot(xhat_history_GN[1,:])
ax[2].plot(xhat_history_GN[2,:]*rnmf.RAD_TO_DEG)
plt.show()
x_init = xhat_history_GN[:,-1]
# %%
