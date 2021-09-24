import numpy as np
import matplotlib.pyplot as plt

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
