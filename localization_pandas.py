#%%
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sensor_fusion as sf
import robot_n_measurement_functions as rnmf
import pathlib
import seaborn as sns
import matplotlib.patches as mpatches
from scipy.linalg import expm
import lsqSolve as lsqS

#%%
parent_path = pathlib.Path('/media/muhammad/Data/')
parent_path = parent_path/'Dropbox/09. Aalto Postdoc/DiddyBorg_experiment'
Camera = sf.Sensor('Camera',sf.CAMERA_COLUMNS,meas_record_file=parent_path/'test-run-camera.csv',is_linear=False,start_index=154)
#%%
df = pd.read_csv(parent_path/'test-run-camera.csv',delimiter=',',index_col=0,
df.index=pd.to_datetime(df.index,dayfirst=True,errors='coerce',unit='s',utc=True)
df.columns = np.array(sf.CAMERA_COLUMNS)[:,0]

# %%
