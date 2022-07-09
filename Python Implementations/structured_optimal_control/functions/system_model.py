import numpy as np
import math

def system_model(N, AV_number, alpha, beta, v_max, s_st,
                 s_go, s_star, gamma_s, gamma_v, gamma_u):
    alpha1 = alpha * v_max / 2 * math.pi / (s_go - s_st) * np.sin(np.pi * (s_star - s_st) / (s_go - s_st))
    alpha2 = alpha + beta
    alpha3 = beta

    C1 = np.array([[0, -1], [0, 0]])
    C2 = np.array([[0, 1], [0, 0]])
    pos1 = 1
    pos2 = N

    # Y = AY + Bu
    A = np.zeros((2 * N, 2 * N))

    for i in range(1, N):
        A[(2 * i - 2): (2 * i), (2 * pos1 - 2): (2 * pos1)] = np.array([[0, -1], [alpha1[i - 1][0], -alpha2[i - 1][0]]])
        A[(2 * i - 2): (2 * i), (2 * pos2 - 2): (2 * pos2)] = np.array([[0, 1], [0, alpha3[i - 1][0]]])
        pos1 = pos1 + 1
        pos2 = np.mod(pos2 + 1, N)


    A[(2 * N - 2): (2 * N), (2 * pos1 - 2): (2 * pos1)] = C1
    A[(2 * N - 2): (2 * N), (2 * pos2 - 2): (2 * pos2)] = C2

    # Controller
    Q = np.zeros((2 * N, 2 * N))
    for i in range(1, N + 1):
        Q[2 * i - 2, 2 * i - 2] = gamma_s
        Q[2 * i - 1, 2 * i - 1] = gamma_v

    B2 = np.zeros((2 * N, AV_number))
    B2[2 * N - 1, AV_number - 1] = 1

    if AV_number == 2:
        AV2_Index = np.floor(N / 2)
        A[(2 * AV2_Index - 2): (2 * AV2_Index), (2 * AV2_Index - 2): (2 * AV2_Index)] = C1
        A[(2 * AV2_Index - 4): (2 * AV2_Index), (2 * AV2_Index - 4): (2 * AV2_Index - 2)] = C2
        B2[2 * AV2_Index - 1, 1] = 1

    B1 = np.zeros((2 * N, N))
    for i in range(1, N + 1):
        B1[2 * i - 1, i - 1] = 1

    R = gamma_u * np.identity(AV_number)

    return A,B1,B2,Q,R