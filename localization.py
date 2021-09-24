#%%
%matplotlib auto
import numpy as np
import matplotlib.pyplot as plt
#%%
# Accompanying code for exercise 4 and 5
# Basics Sensor Fusion , Autumn 2019
# Written by Muhammad.Emzir@Aalto.fi
# Based on MATLAB code of simo.sarkka@Aalto.fi
QRCODE_SIDE_LENGTH = 11.5 #cm
PERCEIVED_FOCAL_LENGTH = 6200/QRCODE_SIDE_LENGTH #pixel
#%%
"""
x is parameters/state/estimate
y is measurement(s)
g is nonlinear measurement models
params is the rest parameters
"""
def Jwls(x,y,g,params):
    delta = y - g(x,params)
    LR = params['LR']
    return np.sum((np.linalg.solve(LR,delta))**2)

#%%
"""
find x in the line points that minimize Jwls
"""
def find_gamma_min(x,dx,line_gammas,J_):
    x_min =  x + line_gammas[0]*dx
    J_min = J_(x_min)
    min_index = 0
    for i in range(1,len(line_gammas)):
        x_new = x + line_gammas[i]*dx
        J_new = J_(x_new)
        if J_new<J_min:
            J_min = J_new
            min_index = i
    
    return line_gammas[min_index]
#%%
"""
implementation of Gradient descent and Gauss Newton
for nonlinear least square
"""
def lsqsolve(y,g,G,x_init,params,method='grad-desc'):
    n_state = x_init.shape[0]
    xhat_history = np.empty((n_state,params['I_max']+1),dtype=np.float64)
    J_history = np.empty(params['I_max']+1,dtype=np.float64)
    Jwls = params['Jwls']
    I_max = params['I_max']
    gamma = params['gamma']
    # R = params['R']
    Rinv = params['Rinv']
    Line_search = params['Line_search']
    n_points = params['Line_search_n_points']

    #make some aliases
    g_ = lambda x: g(x,params)
    G_ = lambda x: G(x,params)
    J_ = lambda x:Jwls(x,y,g,params)
    
    
    if method == 'grad-desc':       
        # dx = np.linalg.solve(R,Gnow).T@delta_y
        # dx_ = lambda x,y,gnow,Gnow: np.linalg.solve(R,Gnow).T@(y-gnow)
        dx_ = lambda x,y,gnow,Gnow: (Gnow.T@Rinv)@(y-gnow)
        gamma_ = lambda x,dx: gamma
    elif method == 'gauss-newton':
        dx_ = lambda x,y,gnow,Gnow: np.linalg.solve(Gnow.T@Rinv@Gnow,Gnow.T@Rinv)@(y-gnow)
        
        if Line_search:
            line_gammas = np.arange(1,n_points+1)/n_points
            gamma_ = lambda x,dx: find_gamma_min(x,dx,line_gammas,J_)
        else:
            gamma_ = lambda x,dx: 1
    elif method == 'levenberg-marquardt':
        lambda_LM = params['lambda_LM']
        scaled_LM = params['scaled_LM']
        gamma_ = lambda x,dx: 1
        if scaled_LM:
            dx_ = lambda x,y,gnow,Gnow: np.linalg.solve(Gnow.T@Rinv@Gnow + lambda_LM*(np.diag(np.diag(Gnow.T@Rinv@Gnow))),Gnow.T@Rinv)@(y-gnow)
        else:
            dx_ = lambda x,y,gnow,Gnow: np.linalg.solve(Gnow.T@Rinv@Gnow + lambda_LM*(np.eye(n_state)),Gnow.T@Rinv)@(y-gnow)

    x = x_init
    xhat_history[:,0] = x
    J_history[0]  = J_(x)
    for i in range(1,I_max+1):
        gnow =  g_(x)
        Gnow = G_(x)
        dx = dx_(x,y,gnow,Gnow)
        gamma =  gamma_(x,dx)      
        x = x+ gamma*dx
        xhat_history[:,i] = x
        J_history[i] = J_(x)

    return xhat_history, J_history

#%%
#Model
"""
"""
def phi_(x_c,y_c,x_i,y_i,psi):
    return np.arctan2((x_c-x_i),(y_i-y_c)) - psi
"""
"""
def dist_(x_c,y_c,x_i,y_i):
    return np.sqrt((x_c-x_i)**2+(y_c-y_i)**2)


"""
g is nonlinear measurement model distance and angle
x is 1x3 vector describing the global position of camera lense, and the attitude of the robot
"""
def g_cam3(x,params):
    x_sensors = params['x_sensors']#x,y position of each QRcodes
    g = np.zeros(2*x_sensors.shape[0])
    x_c = x[0]
    y_c = x[1]
    psi = x[2]
    for i in range(x_sensors.shape[0]):
        # dist = np.sqrt((x_c-x_sensors[i,0])**2+(y_c-x_sensors[i,1])**2)
        # phi = np.arctan2((x_sensors[i,0]-x_c),(x_sensors[i,1]-y_c)) - psi
        g[i*2] = dist_(x_c,y_c,x_sensors[i,0],x_sensors[i,1])
        g[i*2+1] = phi_(x_c,y_c,x_sensors[i,0],x_sensors[i,1],psi)
    
    return g
"""
g is nonlinear measurement model qr code height and x center
x is 1x3 vector describing the global position of camera lense, and the attitude of the robot
"""
def g_cam2(x,params):
    x_sensors = params['x_sensors']#x,y position of each QRcodes
    g = np.zeros(2*x_sensors.shape[0])
    x_c = x[0]
    y_c = x[1]
    psi = x[2]
    for i in range(x_sensors.shape[0]):
        dist = dist_(x_c,y_c,x_sensors[i,0],x_sensors[i,1])
        phi = phi_(x_c,y_c,x_sensors[i,0],x_sensors[i,1],psi)
        g[i*2] = PERCEIVED_FOCAL_LENGTH*QRCODE_SIDE_LENGTH/dist#the QR-code pixel height
        g[i*2+1] = PERCEIVED_FOCAL_LENGTH*np.tan(phi)#the QR-code center x
    
    return g

#g For estimating x_c and y_c from height of QR-code only
def g_cam1(x,params):
    x_sensors = params['x_sensors']#x,y position of each QRcodes
    g = np.zeros(x_sensors.shape[0])
    x_c = x[0]
    y_c = x[1]
    # psi = x[2]
    for i in range(x_sensors.shape[0]):
        dist = dist_(x_c,y_c,x_sensors[i,0],x_sensors[i,1])
        # phi = np.arctan2((x_sensors[i,0]-x_c),(x_sensors[i,1]-y_c)) - psi
        g[i] = PERCEIVED_FOCAL_LENGTH*QRCODE_SIDE_LENGTH/dist#the QR-code pixel height
        # g[i*2+1] = PERCEIVED_FOCAL_LENGTH*np.tan(phi)#the QR-code center x
    
    return g

#%%
"""
G is The Jacobian of g
x is 1xn vector
"""
def G_cam3(x,params):
    x_sensors = params['x_sensors']#x,y position of each QRcodes
    G = np.zeros((2*x_sensors.shape[0],3))
    x_c = x[0]
    y_c = x[1]
    psi = x[2]
    for i in range(x_sensors.shape[0]):
        dist = dist_(x_c,y_c,x_sensors[i,0],x_sensors[i,1])
        phi = phi_(x_c,y_c,x_sensors[i,0],x_sensors[i,1],psi)
        G[i*2,:] = np.array([(x_c-x_sensors[i,0]),(y_c-x_sensors[i,1]),0])/dist
        G[i*2+1,:] = np.array([(y_c-x_sensors[i,1])/(dist*dist),(x_sensors[i,0]-x_c)/(dist*dist),-1])
    return G

#G For estimating x_c and y_c from height of QR-code only
def G_cam2(x,params):
    x_sensors = params['x_sensors']#x,y position of each QRcodes
    G = np.zeros((2*x_sensors.shape[0],3))
    x_c = x[0]
    y_c = x[1]
    psi = x[2]
    for i in range(x_sensors.shape[0]):
        dist = dist_(x_c,y_c,x_sensors[i,0],x_sensors[i,1])
        phi = phi_(x_c,y_c,x_sensors[i,0],x_sensors[i,1],psi)
        G[i*2,:] = -np.array([(x_c-x_sensors[i,0]),(y_c-x_sensors[i,1]),0])*PERCEIVED_FOCAL_LENGTH*QRCODE_SIDE_LENGTH/(dist*dist*dist)
        G[i*2+1,:] = np.array([(y_c-x_sensors[i,1])/(dist*dist),(x_sensors[i,0]-x_c)/(dist*dist),-1])*PERCEIVED_FOCAL_LENGTH/(np.cos(phi)**2)
    return G

#G For estimating x_c and y_c from height of QR-code only
def G_cam1(x,params):
    x_sensors = params['x_sensors']#x,y position of each QRcodes
    G = np.zeros((x_sensors.shape[0],2))
    x_c = x[0]
    y_c = x[1]
    # psi = x[2]
    for i in range(x_sensors.shape[0]):
        dist = np.sqrt((x_c-x_sensors[i,0])**2+(y_c-x_sensors[i,1])**2)
        # phi = np.arctan2((x_sensors[i,0]-x_c),(x_sensors[i,1]-y_c)) - psi
        G[i] = -np.array([(x_c-x_sensors[i,0]),(y_c-x_sensors[i,1])])*PERCEIVED_FOCAL_LENGTH*QRCODE_SIDE_LENGTH/(dist*dist*dist)
        # G[i*2+1,:] = np.array([(y_c-x_sensors[i,1])/(dist*dist),(x_sensors[i,0]-x_c)/(dist*dist),-1])*PERCEIVED_FOCAL_LENGTH/(np.cos(phi)**2)
    return G
#%%
g = g_cam3

if g == g_cam1:
    G = G_cam1
elif g == g_cam2:
    G = G_cam2
else:
    G = G_cam3
#%%
#target position
# x = np.array([5,3.7])
if g == g_cam1:
    x = np.array([48.08358732, 58.76823964])
else:
    x = np.array([48.08358732, 58.76823964,np.pi/2])   #<-- localization
    x = np.array([17, 60,np.pi/2])   #<-- tracking



#%%
#selection
sel = 4
#target position
if sel == 1 or sel==2:
    x_init = np.array([4,1.4])
elif sel == 3:
    x_init = np.array([-5,0])
elif sel ==4:
    if g == g_cam1:
        x_init = x+2.5*np.random.randn(2)
    else:
        x_init = x+0.5*np.random.randn(3)
else:
    x_init = 2.5*np.random.randn(2)

#%%
#sensor position
#QR SENSOR 4 28 29 30 36
# x_sensors = np.array([[0,35.65],[0,71.55],[0,59.55],[0,47.55],[0,84]])
#QR SENSOR 21 25 26 27 31
# x_sensors = np.array([[37.25,120],[49.75,120],[61.75,120],[73.75,120],[85.95,120]])

#variance of sensors
if g == g_cam1:
    sigma2_r = np.array([1,1,1,1,1])
elif g == g_cam2:
    sigma2_r = np.array([1,1,1,1,1,1,1,1,1,1])
else:
    sigma2_r = np.array([1,0.1,1,0.1,1,0.1,1,0.1,1,0.1])

R = np.diag(sigma2_r)
#Maximum number of iteration for the numerical solvers
I_max = 100

#%%
# method = 'grad-desc' #<-- green
# method = 'gauss-newton' #<-- red
# method = 'levenberg-marquardt' #<-- black

Rinv = np.linalg.inv(R)
Gnow = G(x,{'x_sensors':x_sensors})
tau_LM = 1
lambda_LM = np.max(np.diag(Gnow.T@Rinv@Gnow))

# if method== 'grad-desc':
#     #gradient descent step
#     gamma = 0.1
# else:
#     gamma = 1
gamma = 1
params_default = {'x_sensors':x_sensors,
        'R':R,
        'LR':np.linalg.cholesky(R).T,#cholesky factorization of a matrix (chol(a) in matlab returns an upper triangular matrix, but linalg.cholesky(a) returns a lower triangular matrix)
        'Rinv':Rinv,
        'gamma':gamma,
        'I_max':I_max,
        'Line_search':False,
        'Line_search_n_points':10,
        'lambda_LM':lambda_LM,
        'scaled_LM':False,
        'Jwls':Jwls
        }
#%%
#Generate Data 
r = np.sqrt(sigma2_r)*np.random.randn()
# y = g(x,params_default) + r

#QR SENSOR 21 25 26 27 31 <- localization 3
# y_full=np.array([[208.003268,107.0555556,60.81381495,21.0972383],
# [99.032,102.968,61.21955019,10.40853709],
# [-3.003558719,98.79715302,61.8799777,-0.319198183],
# [-106.0607143,100.1321429,61.30454585,-11.1294193],
# [-211.0071174,103.9928826,61.97378528,-21.3745879]
# ])

#QR SENSOR 4 28 29 30 36 <- localization 2
y_full=np.array([[124, 257,	50,	0.444829197],
[123.0128755,	-117.9227468,	50.40127011,	-0.215336346],
[121.9,	7.995652174,	50.87278725,	0.014829558],
[121.976834,	133.003861,	50.8294018,	0.241871044],
[121.5597015,	-241.0447761,	51.00462179,	-0.420438941]
])


if g == g_cam1:
    y = y_full[:,1]
elif g == g_cam2:
    y = y_full[:,:2].ravel()
else:
    y = y_full[:,2:].ravel()

# y = np.array([50,-0.444829,50.81967213,-0.241864296,51.23966942,-0.014837621,50.40650407,0.213702411,51.23966942,0.420369768])
# y = PERCEIVED_FOCAL_LENGTH*QRCODE_SIDE_LENGTH/np.array([48.70218579,49.1046832,48.30623306,49.1046832,47.91666667])

#%%
# methods = ['levenberg-marquardt']
methods = ['grad-desc','gauss-newton','gauss-newton','levenberg-marquardt','levenberg-marquardt']
#methods = ['grad-desc','gauss-newton','gauss-newton']
params_GD = params_default.copy();params_GD['gamma']=0.1
params_GN = params_default.copy()
params_GN_LS = params_default.copy();params_GN_LS['Line_search']=True
params_LM = params_default.copy()
params_LM_SCL = params_default.copy();params_LM_SCL['scaled_LM']=True

params_list = [params_GD, params_GN,params_GN_LS, params_LM,params_LM_SCL]
# params_list = [params_LM_SCL]
for method,params in zip(methods,params_list):
    if method== 'grad-desc':
        #gradient descent step
        xhat_history_GD, J_history_GD = lsqsolve(y,g,G,x_init,params,method=method)
    elif method == 'gauss-newton':
        if not params['Line_search']:
            xhat_history_GN, J_history_GN = lsqsolve(y,g,G,x_init,params,method=method)
        else:
            xhat_history_GN_LS, J_history_GN_LS = lsqsolve(y,g,G,x_init,params,method=method)
    elif method == 'levenberg-marquardt':
        if not params['scaled_LM']:
            xhat_history_LM, J_history_LM = lsqsolve(y,g,G,x_init,params,method=method)
        else:
            xhat_history_LM_SCL, J_history_LM_SCL = lsqsolve(y,g,G,x_init,params,method=method)

#%%
# """ contour plotting of result
# """
# if sel == 1 or sel==2:
#     #data selection 1
#     X_grid,Y_grid = np.meshgrid(np.linspace(0.,10,40),np.linspace(0.,10.,40))
# elif sel==2:
#     #data selection 2
#      X_grid,Y_grid = np.meshgrid(np.linspace(4.,6,20),np.linspace(3.5,6.,20))
# elif sel==3:
#     #data selection 3
#      X_grid,Y_grid = np.meshgrid(np.linspace(-10,10,20),np.linspace(-10,16,20))
# elif sel==4 :
#     #data selection 3
#      X_grid,Y_grid,Psi_grid = np.meshgrid(np.linspace(0,50,30),np.linspace(30,100,30),np.linspace(np.pi/2,3*np.pi/2,30))
# else:
#     X_grid,Y_grid = np.meshgrid(np.linspace(-10,10,20),np.linspace(-10,16,20))
     
# J_ = lambda x:Jwls(x,y,g,params)

# Jxy = np.zeros_like(X_grid,dtype=np.float64)
# for i in range(X_grid.shape[0]):
#     for j in range(X_grid.shape[1]):
#         for k in range(X_grid.shape[2]):
#             x1 = X_grid[i,j.k]
#             x2 = Y_grid[i,j,k]
#             psi = Psi_grid[i,j,k]
#             Jxy[i,j] = J_(np.array([x1,x2,psi]))

# plt.clf()
# plt.rc('text', usetex=True)
# plt.rc('font', family='serif')
# fig, ax = plt.subplots()
# CS = ax.contour(X_grid, Y_grid, Jxy,levels=40,linewidth=0.5)
# #ax.plot(xhat_history[0,:],xhat_history[1,:],'--r^',linewidth=0.5)
# ax.plot(x_sensors[:,0],x_sensors[:,1],'ko',linewidth=0.5)
# ax.plot(x[0],x[1],'-bo',markersize=12)
# ax.clabel(CS, inline=1, fontsize=10)
# ax.set_title(r'$J_{wls}$')
# ax.set_xlabel(r'$x$')
# ax.set_ylabel(r'$y$')
# #%%
# for method,params in zip(methods,params_list):
#     if method== 'grad-desc':
#         #gradient descent step
#         ax.plot(xhat_history_GD[0,:],xhat_history_GD[1,:],'--g^',linewidth=0.5)
#     elif method == 'gauss-newton':
#         if not params['Line_search']:
#             ax.plot(xhat_history_GN[0,:],xhat_history_GN[1,:],'--r^',linewidth=0.5)
#         else:
#             ax.plot(xhat_history_GN_LS[0,:],xhat_history_GN_LS[1,:],'-.ro',linewidth=0.5)
#     elif method == 'levenberg-marquardt':
#         if not params['scaled_LM']:
#             ax.plot(xhat_history_LM[0,:],xhat_history_LM[1,:],'--k^',linewidth=0.5)
#         else:
#             ax.plot(xhat_history_LM_SCL[0,:],xhat_history_LM_SCL[1,:],'-.ko',linewidth=0.5)

# ax.plot(xhat_history_GD[0,:],xhat_history_GD[1,:],'--g^',linewidth=0.5)
# ax.plot(xhat_history_GN[0,:],xhat_history_GN[1,:],'--r^',linewidth=0.5)
# #%%
plt.figure()
for method,params in zip(methods,params_list):
    if method== 'grad-desc':
        #gradient descent step
        plt.loglog(J_history_GD,'--g^',linewidth=0.5)
    elif method == 'gauss-newton':
        if not params['Line_search']:
            plt.loglog(J_history_GN,'--r^',linewidth=0.5)
        else:
            plt.loglog(J_history_GN_LS,'--ro',linewidth=0.5)
    elif method == 'levenberg-marquardt':
        if not params['scaled_LM']:
            plt.loglog(J_history_LM,'--k^',linewidth=0.5)
        else:
            plt.loglog(J_history_LM_SCL,'--ko',linewidth=0.5)


#%%
