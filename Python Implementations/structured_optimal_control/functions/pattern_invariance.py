import numpy as np
def pattern_invariance(S):
#Generate a maximally sparsity-wise invariant (MSI) subplace with respect to X
# See Section IV of the following paper
# "On Separable Quadratic Lyapunov Functions for Convex Design of Distributed Controllers"

    m = S.shape[0]
    n = S.shape[1]
    X = np.ones((n,n))

    # Analytical solution with complexity mn^2
    for i in range(m):
        for k in range(n):
            if S[i-1, k-1] == 0:
                for j in range(n):
                    if S[i - 1, j - 1] == 1:
                        X[j - 1, k - 1] = 0

    # symmetric part
    Xu = np.triu(X.conj().transpose()) * np.triu(X)
    X = Xu + Xu.conj().transpose()
    for i in range(len(X)):
        for j in range(len(X[0])):
            if(X[i][j] != 0) :
                X[i][j] = 1


    return X