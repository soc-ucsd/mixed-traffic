##This code is adapted from MATLAB code https://github.com/wangjw18/mixed-traffic

from ring_traffic_model import *
import cvxpy as cp
import mosek as msk


def lqr_sdp(N,s_star,gamma_s,gamma_v,gamma_u,AV_number):
    
    A, B, Q, R = ring_traffic_model(N,s_star,gamma_s,gamma_v,gamma_u,AV_number)
    
    n = 2*N
    m = AV_number
    
    epsilon = 1e-5
    
    H = np.identity(n)
    for i in range(0,N):
        H[2*i,2*i] = 0
    
    
    #X = cp.Variable((n,n), symmetric=True)
    #Z = cp.Variable((m,n))
    #Y = cp.Variable((m,m), symmetric=True)
    W = cp.Variable((m+n,m+n), symmetric=True)
    #                W[0:m,0:m] == Y,
    #                W[m:,0:m] == Z,
    #                W[m:,m:] == X,
    
    objective    = cp.Minimize( cp.trace( Q@W[m:,m:] ) + cp.trace( R@W[0:m,0:m] ) )
    constraints  = [(A@W[m:,m:] - B@W[0:m,m:]) + (A@W[m:,m:] - B@W[0:m,m:]).T + H@(H).T << 0]
    constraints += [ W[m:,m:] - epsilon*np.identity(n) >> 0]
    constraints += [W >> 0]
    
    # constraints  = [-((A@X - B@Z) + (A@X - B@Z).T + H@(H).T) >> 0]
    # constraints += [X - epsilon*np.identity(n) >> 0]
    # constraints += [np.block([[Y, Z], [Z.T, X]]) >> 0]
    
    prob = cp.Problem(objective, constraints)
    prob.solve(solver = cp.MOSEK)
    
    Xd = W[m:,m:].value
    Zd = W[0:m,m:].value

    
    K = Zd@np.linalg.inv(Xd)
    
    return K