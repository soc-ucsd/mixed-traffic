#This code is adapted from MATLAB code https://github.com/wangjw18/mixed-traffic

from math import pi
from math import floor
from array import *
import numpy as np

def ring_traffic_model(N,s_star,gamma_s,gamma_v,gamma_u,AV_number):

    # Generate a dynamic model for 
    #OVM

    OVM = 1

    alpha  = 0.6
    beta   = 0.9
    v_max  = 30
    s_st   = 5
    s_go   = 35


    # General
    if OVM:
        alpha1 = alpha*v_max/2*pi/(s_go-s_st)*np.sin(pi*(s_star-s_st)/(s_go-s_st))
        alpha2 = alpha+beta
        alpha3 = beta
    else:
        alpha1 = 2.6;
        alpha2 = 3;
        alpha3 = 2;


    A1 = np.array([[0,-1], [alpha1,-alpha2]])
    A2 = np.array([[0,1], [0,alpha3]])
    C1 = np.array([[0,-1], [0,0]])
    C2 = np.array([[0,1], [0,0]])


    pos1 = 0
    pos2 = N-1
    

    A = np.zeros((2*N,2*N)) 

    for i in range(N-1):
        A[(2*i):(2*i+2), (2*pos1):(2*pos1+2)] = A1
        A[(2*i):(2*i+2), (2*pos2):(2*pos2+2)] = A2
        
        pos1 = pos1+1
        pos2 = (pos2+1) % (N)
    
    A[(2*N-2):(2*N), (2*pos1):(2*pos1+2)] = C1
    A[(2*N-2):(2*N), (2*pos2):(2*pos2+2)] = C2
    
    
    #Controller

    Q = np.zeros((2*N, 2*N))    
    
    for i in range(N):
        Q[2*i,2*i] = gamma_s
        Q[2*i+1,2*i+1] = gamma_v
    

    B = np.zeros((2*N,AV_number))    #[[0 for col in range(AV_number)] for row in range(2*N)]  
    
    B[2*N-1,0] = 1
    if AV_number == 2:
        AV2_Index = floor(N/2);
        A[(2*AV2_Index-2):(2*AV2_Index), (2*AV2_Index-2):(2*AV2_Index)] = C1
        A[(2*AV2_Index-2):(2*AV2_Index), (2*AV2_Index-4):(2*AV2_Index-2)] = C2
        B[2*AV2_Index-1,1] = 1
    

    R = gamma_u*np.identity(AV_number)
    
    return A, B, Q, R