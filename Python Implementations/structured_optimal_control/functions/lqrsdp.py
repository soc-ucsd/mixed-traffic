##This code is adapted from MATLAB code https://github.com/wangjw18/mixed-traffic
import cvxpy as cp
import numpy as np

def lqrsdp(A, B1, B, Q, R):    
    n = len(A)
    m = len(B[0])
    
    epsilon = 1e-5
        
    #X = cp.Variable((n,n), symmetric=True)
    #Z = cp.Variable((m,n))
    #Y = cp.Variable((m,m), symmetric=True)
    W = cp.Variable((m+n,m+n), symmetric=True)
    #                W[0:m,0:m] == Y,
    #                W[m:,0:m] == Z,
    #                W[m:,m:] == X,
    
    objective    = cp.Minimize(cp.trace(Q @ W[m:,m:]) + cp.trace(R @ W[0:m,0:m]))
    constraints  = [(A @ W[m:,m:] - B @ W[0:m,m:]) + (A @ W[m:,m:] - B @ W[0:m,m:]).T + B1 @ (B1).T << 0]
    constraints += [W[m:,m:] - epsilon * np.identity(n) >> 0]
    constraints += [W >> 0]
    
    # constraints  = [-((A@X - B@Z) + (A@X - B@Z).T + H@(H).T) >> 0]
    # constraints += [X - epsilon*np.identity(n) >> 0]
    # constraints += [np.block([[Y, Z], [Z.T, X]]) >> 0]
    
    prob = cp.Problem(objective, constraints)
    prob.solve(solver = cp.MOSEK, verbose = True)
    
    Xd = W[m:,m:].value
    Zd = W[0:m,m:].value

    K = Zd @ np.linalg.inv(Xd)
    
    return K
