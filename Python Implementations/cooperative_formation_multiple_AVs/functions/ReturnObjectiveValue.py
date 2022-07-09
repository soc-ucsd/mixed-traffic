import cvxpy as cp
import numpy as np

def ReturnObjectiveValue(AV_ID, N, alpha1, alpha2, alpha3, gammaType) :
    # Generate the system model and the optimal objective value
    
    if(gammaType == 1) :
    #S1
        gamma_s = 0.01
        gamma_v = 0.05
        gamma_u = 0.1
        
    elif(gammaType == 2) :
    #S2
        gamma_s = 0.03
        gamma_v = 0.15
        gamma_u = 0.1
        
    elif(gammaType == 3) :
    #S3
        gamma_s = 0.05
        gamma_v = 0.25
        gamma_u = 0.1
         
    elif(gammaType == 4) :
    #S4
        gamma_s = 0.03
        gamma_v = 0.15
        gamma_u = 1
    
    elif(gammaType == 5) :
        gamma_s = 1
        gamma_v = 1
        gamma_u = 0
        
    elif(gammaType == 9999) :
        gamma_s = 0.01
        gamma_v = 0.05
        gamma_u = 1e-6

    AV_number = np.count_nonzero(AV_ID)
    A1 = [[0,-1],[alpha1,-alpha2]]
    A2 = [[0,1],[0,alpha3]]
    C1 = [[0,-1],[0,0]]
    C2 = [[0,1],[0,0]]
    
    A = np.zeros((2 * N, 2 * N))
    B = np.zeros((2 * N, AV_number))
    Q = np.zeros((2 * N, 2 * N))

    for i in range(1, N + 1) :
        Q[2 * i - 2, 2 * i - 2] = gamma_s
        Q[2 * i - 1, 2 * i - 1] = gamma_v
    
    R = gamma_u * np.eye(AV_number)

    A[0:2,0:2] = A1
    A[0:2,(2 * N - 1) - 1:2 * N] = A2

    for i in range(2, N + 1) : 
        A[(2 * i - 1) - 1 : (2 * i) , (2 * i - 1) - 1 : (2 * i)] = A1
        A[(2 * i - 1) - 1 : (2 * i) , (2 * i - 3) - 1 : (2 * i - 2)] = A2

    if (alpha2 ** 2) - (alpha3 ** 2) - (2 * alpha1) > 0 :
        stability_condition_bool = True
    else :
        stability_condition_bool = False

    temp_x = np.nonzero((np.real(np.linalg.eig(A)[0]) > 0.001)[0])
    if temp_x == 0 :
        stable_bool = True
    else :
        stable_bool = False

    k = 1
    for i in range(1, N + 1):
        if AV_ID[i - 1] == 1:
            if i == 1:
                A[1 - 1 : 2, 1 - 1 : 2] = C1
                A[1 - 1 : 2, (2 * N - 1 - 1) : 2 * N] = C2
            else:
                A[(2 * i - 2) : (2 * i) , (2 * i - 2) : (2 * i)] = C1
                A[(2 * i - 2) : (2 * i) , (2 * i - 4) : (2 * i - 2)] = C2
            B[2 * i - 1, k - 1] = 1
            k = k + 1

    # Call Yalmip to calculate the optimum
    epsilon   = 1e-5

    n = len(A)  # number of states
    m = len(B[0]) # number of inputs

    # assume each vehicle has a deviation
    B1 = np.eye(n)
    B1[0 : n : 2, 0 : n : 2] = 0

    # B1 = B;
    # assume disturbance is the same as input

    # variables
    # X = cp.Variable(shape = (n,n))
    # Z = cp.Variable(shape = (m,n))
    # Y = cp.Variable(shape = (m,m))
    # constraints = [M + M.T + (B1 @ B1.T) <= 0,\
    #    X - epsilon *  np.eye(n) >= 0,\
    #    cp.vstack((cp.hstack((Y,Z)), cp.hstack((Z.T,X)))) >= 0]

    S = cp.Variable((m + n, m + n), symmetric = True)

    # constraints
    constraints  = [(A @ S[m:,m:] - B @ S[0 : m,m:]) + (A @ S[m:,m:] - B @ S[0 : m,m:]).T + B1 @ B1.T << 0 ,\
         S[m:,m:] - epsilon * np.eye(n) >> 0,\
         S >> 0]

    # print(cp.installed_solvers())

    obj = cp.trace(Q @ S[m:,m:]) + cp.trace(R @ S[0:m,0:m])
    problem = cp.Problem(cp.Minimize(obj), constraints)
    problem.solve(solver = cp.MOSEK, verbose = True)

    Xd = S[m:,m:].value
    Zd = S[0:m,m:].value
    Yd = S[0:m,0:m].value

    K = Zd @ np.linalg.inv(Xd)
    Obj = cp.trace(Q @ Xd) + cp.trace(R @ Yd)

    np.set_printoptions(threshold=np.inf)
    np.set_printoptions(linewidth=100)
    np.set_printoptions(precision=4)
    print(K)

    return Obj,stable_bool,stability_condition_bool,K
