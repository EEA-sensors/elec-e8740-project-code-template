#%%
#%matplotlib auto
import numpy as np
import matplotlib.pyplot as plt
import sensor_fusion as sf
import robot_n_measurement_functions as rnmf
import pathlib
import seaborn as sns
import matplotlib.patches as mpatches
from scipy.linalg import expm
import lsqSolve as lsqS
import pathlib
sns.set()
#%%
parent_path = pathlib.Path.home()#('/media/muhammad/Data/')
parent_path = parent_path/'Dropbox/09. Aalto Postdoc/DiddyBorg_experiment'
Camera = sf.Sensor('Camera',sf.CAMERA_COLUMNS,meas_record_file=parent_path/'test-run-camera.csv',is_linear=False,start_index=154)
#%%

x_init = np.array([17,60,0])
x = np.zeros((Camera.meas_record.shape[0]//3,3),dtype=np.float)
x[0,:] = x_init
t = np.zeros(x.shape[0])
t[0] = Camera.time[0]

R_one_diag = np.array([2,20])
#R_one_diag = np.array([2])

I_max=50
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
# for i in range(1,x.shape[0]-1):
i=0
while(Camera.current_sample_index<Camera.time.shape[0] and i<x.shape[0]-1):
    i +=1    
    # print('EKF')
    t = Camera.current_time
    y_raw = Camera.get_measurement()
    n_qr_codes = y_raw.shape[0]
    
    if n_qr_codes < 3:
        x[i,:] = x[i-1,:]
        continue
    
    # dist = y_raw[:,5]
    # direct = y_raw[:,-1]*rnmf.DEG_TO_RAD
    # y_raw[:,5] = dist/np.cos(direct)

    weight = y_raw[:,3]
    height = y_raw[:,4]
    c_x = y_raw[:,1]

    dist = rnmf.QRCODE_SIDE_LENGTH*rnmf.PERCEIVED_FOCAL_LENGTH/height
    direct = np.arctan2(c_x,rnmf.PERCEIVED_FOCAL_LENGTH) 
    angle_qr = np.arccos(np.minimum(weight,height)/height)

    corrected_dist = dist/np.cos(direct) + 0.5*rnmf.QRCODE_SIDE_LENGTH*np.sin(angle_qr)
    y_raw[:,5] = corrected_dist#dist/np.cos(direct)
    y = y_raw[:,5:].flatten()

    qr_pos = rnmf.QRCODE_LOCATIONS[y_raw[:,0].astype('int'),1:]
    params_LSQ['x_sensors'] = qr_pos

    
    R = np.diag(np.kron(np.ones(n_qr_codes),R_one_diag))
    params_LSQ['R'] = R
    params_LSQ['LR'] = np.linalg.cholesky(R).T
    params_LSQ['Rinv'] = np.diag(1/np.diag(R))
    xhat_history_GN, J_history_GN = lsqS.lsqsolve(y,rnmf.h_cam,rnmf.H_cam,x[i-1,:],params_LSQ,method='gauss-newton')
    x[i,:] = xhat_history_GN[:,-1]


#%%
# plt.figure()
#plt.plot(x[:,0],x[:,1],'-ok',linewidth=0.5,markersize=2)
skip=5
end_index=(x.shape[0]-1)
fig, ax = plt.subplots()
ax.plot(x[:end_index:skip,0], x[:end_index:skip,1])
q = ax.quiver(x[:end_index:skip,0], x[:end_index:skip,1], -np.sin(x[:end_index:skip,2]), np.cos(x[:end_index:skip,2]),headwidth=1,width=0.0051,alpha=0.8,color='blue')
p = mpatches.Circle((x[0,0], x[0,1]), 1,color='red')
ax.add_patch(p)
p = mpatches.Circle((x[end_index,0], x[end_index,1]), 1,color='black')
ax.add_patch(p)
ax.plot(x[:end_index,0],x[:end_index,1],'-r',linewidth=4,alpha=0.5)


# %%
