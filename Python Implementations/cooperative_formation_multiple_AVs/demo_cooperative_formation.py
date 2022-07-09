import math
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.animation import FuncAnimation
from ReturnObjectiveValue import *
from getAVID import *

# In[1] 
''' Key Parameters'''

N = 20
AV_number = 4 # 0 or 1 or 2 or 4

platoon_bool = 1 # 0 or 1

# Position of the perturbation
brakeID = 4


# In[2] 
''' Parameters '''

if AV_number == 0:
    mix = 0
    ActuationTime = 9999
else:
    mix = 1

ID = np.zeros([N]) #0. Manually Driven  1. Controller

if mix:
    ActuationTime = 0
    ID = getAVID(ID, AV_number, platoon_bool)

#Controller Parameter
gammaType = 2

v_star = 15

# OVM parameter
s_star = 20
v_max  = 30
s_st   = 5
s_go   = 35

'''%%%%% Type1 %%%%%%%'''
alpha  = 0.6
beta   = 0.9

'''%%%%%%%%% Type2 %%%%%%%%%'''
#     alpha  = 1.0
#     beta   = 1.5



# In[3] 
'''Other Parameters'''

acel_max = 2
dcel_max = -5

'''%Driver Model: OVM'''

'''% safe distance for collision avoidance'''
sd = 8 # minimum value is zero since the vehicle length is ignored

#Simulation
TotalTime = 100
Tstep = 0.001
NumStep = int(TotalTime/Tstep)
#Scenario
Circumference = s_star*N


#Initial State for each vehicle
S = np.zeros((NumStep,N,3))
dev_s = 0
dev_v = 0
co_v = 1.0
v_ini = co_v * v_star #Initial velocity
#from -dev to dev

var1 = np.linspace(Circumference, s_star, N)
var2 = np.random.rand(N)*2*dev_s-dev_s    
S[0, :, 0] = var1 + var2
    
var1 = v_ini*np.ones([N])
var2 = (np.random.rand(N)*2*dev_v-dev_v)    
S[0, :, 1] =  var1 + var2

# In[4] 
#Velocity Difference
V_diff = np.zeros([NumStep,N])
#Following Distance
D_diff = np.zeros([NumStep,N])
temp = np.zeros([N])
#Avg Speed
V_avg = np.zeros((NumStep,1))

X = np.zeros((2*N,NumStep))


# In[4] 
##Controller

alpha1 = alpha*v_max/2* math.pi /(s_go-s_st)* math.sin (math.pi*(s_star-s_st)/(s_go-s_st))
alpha2 = alpha+beta
alpha3 = beta

# In[apply f] 
if mix:
    Obj,stable_bool,stability_condition_bool,K = ReturnObjectiveValue(ID,N,alpha1,alpha2,alpha3,gammaType)


# In[5] 

#Simulation
for k in range(0,NumStep-2):
    
    #Car in front velocity
    temp[1:] = S[k,:-1,1]
    temp[0] = S[k,-1,1]
    V_diff[k,:] = temp-S[k,:,1]
    
    temp[0]=S[k,-1,0]+ Circumference
    temp[1:] = S[k,:-1,0]

    D_diff[k,:] = temp-S[k,:,0]

    cal_D = D_diff[k,:]

    ## might be different
    cal_D[cal_D>s_go] = s_go
    cal_D[cal_D<s_st] = s_st


    acel2 = math.pi*(cal_D-s_st)/(s_go-s_st)
    acel1 = (1-np.cos(acel2))
    acel3 = np.zeros(N)
    
    
    acel = alpha*(v_max/2*acel1-S[k,:,1])+beta*V_diff[k,:] + acel3
    acel[acel>acel_max] = acel_max
    acel[acel<dcel_max] = dcel_max
    
    
    #SD as ADAS to prevent crash
    temp[1:] = S[k,:-1,1]
    temp[0] = S[k,-1,1]
    acel_sd = (S[k,:,1]**2-temp**2)/2/(D_diff[k,:]-sd)
    #if (k%100==0):
        #print(D_diff[k,:])
        #print(temp)
        #print(acel_sd)
        #print(" ")
    acel[acel_sd>abs(dcel_max)] = dcel_max
    
    S[k,:,2] = acel

    if mix:
        AV_position = np.nonzero(ID == 1)
        if (k> ActuationTime/Tstep):
            X[np.arange(0,2*N,2),k] = D_diff[k,:]-s_star
            X[np.arange(1,2*N,2),k] = S[k,:,1]-v_star
            u = -K@X[:,k]    
             
            #error might
            t_x = np.nonzero(u>acel_max)
            if np.all(t_x==0):
                u[u>acel_max] = acel_max
            elif np.all((np.nonzero(u<dcel_max))==0):
                u[u<dcel_max] = dcel_max
                
            for i_AV in range(0,AV_number):
                id_AV = AV_position[0][i_AV]
                flag = (pow(S[k,id_AV,1],2)-pow(S[k,id_AV-1,1],2)) / 2 / (S[k,id_AV-1,0]-S[k,id_AV,0]-sd) > abs(dcel_max)
                if (flag.any()):
                    u[i_AV] = dcel_max
                S[k,id_AV,2] = u[i_AV]
                


    if (k*Tstep>20) and (k*Tstep<22):
        S[k,brakeID,2]=-5

    S[k+1,:,1] = S[k,:,1] + Tstep*S[k,:,2]
    S[k+1,:,0] = S[k,:,0] + Tstep*S[k,:,1]
    
for k in range(NumStep):
    V_avg[k] = np.mean(S[k,:,1])


# In[6] 
#Plot
Lwidth = 1.2
Wsize = 20

# In[7] 
# Velocity

#Settling Time
final_velocity = V_avg[NumStep-2]
above_1_percent = final_velocity*1.03
below_1_percent = final_velocity*0.97

settling_time = 0
for k in range(NumStep-2,0,-1):
    for j in range(N):
        if (S[k,j,1] > above_1_percent) or (S[k,j,1] < below_1_percent):
            settling_time = k/100
            break
    if (settling_time != 0):
        break

print("Settling Time within 3% is",settling_time, "s")

#Maximum Spacing in fron of AV
max_space = 0
for k in range(NumStep):
    curr_space = S[k,-2,0]-S[k,-1,0]
    if ( curr_space > max_space):
        max_space = curr_space

print("Maximum Spacing in front of AV is ", round(max_space,2) )

#Average settled velocity
print("Average settled velocity is ", round(np.mean(S[(int((0.9*TotalTime)/Tstep)):,:,1]),2), " m/s")


spacing_or_velocity = 1 # 0 or 1 
#Display data

fig = plt.figure()
x = np.arange(0,NumStep)

# syntax for 3-D projection

#y = np.linspace(0,20,20)

if spacing_or_velocity == 0:
    ax = plt.axes(projection ='3d')
    for i in range(N):
        z = np.ones(NumStep-1)*i

        # if i==N-1 and mix==1:
        if ID[i] == 1:
            ax.plot3D(z, x[:-1], S[:-1,i-1,0] - S[:-1,i,0], 'red', linewidth=0.5)
            continue

        if i==0:
            ax.plot3D(z, x[:-1], S[:-1,i-1,0] - S[:-1,i,0] + Circumference, 'blue', linewidth=0.5)
            continue

        ax.plot3D(z, x[:-1], S[:-1,i-1,0] - S[:-1,i,0], 'blue', linewidth=0.5)


    ax.set_yticks(np.linspace(0,NumStep,5))
    ax.set_yticklabels(np.linspace(0,TotalTime,5))
    ax.set_xlabel("Vehicle ID")
    ax.set_ylabel("Time")
    ax.set_zlabel("Spacing from vehicle ahead")
    title = "N=" + str(N) + ", mix=" + str(mix)
    ax.set_title(title)
    plt.show()

if spacing_or_velocity == 1:
    ax = plt.axes(projection ='3d')
    for i in range(N):
        z = np.ones(NumStep-1)*i

        # if i==N-1 and mix==1:
        if ID[i] == 1:
            ax.plot3D(z, x[:-1], S[:-1,i,1], 'red', linewidth=0.5)
            continue

        ax.plot3D(z, x[:-1], S[:-1,i,1], 'blue', linewidth=0.5)


    ax.set_yticks(np.linspace(0,NumStep,5))
    ax.set_yticklabels(np.linspace(0,TotalTime,5))
    ax.set_xlabel("Vehicle ID")
    ax.set_ylabel("Time")
    ax.set_zlabel("Vehicle Velocity")
    title = "N=" + str(N) + ", mix=" + str(mix) 
    ax.set_title(title)
    plt.show()


# Animation
if spacing_or_velocity != 2:
    velUpperBound = 15 # color
    velLowerBound = 8 # color
    vehicleSize = 10.5 # MarkerSize

    fig = plt.figure(figsize=(16, 9), dpi=80) # ADD A PARAMETER TO NUMERICALLY IDENTIFY FIGURE
    fig.set_facecolor('w') 
    axs1 = plt.subplot(121)
    axs1.set_aspect('equal')
    plt.axis('off')

    R = Circumference/2/math.pi * 2
    position = [None] * N

    def init():
        #Vehicles
        for id in range(N) :
            if not mix :
                position[id] = plt.plot(R * np.cos(S[0, id, 0] / Circumference * 2 * math.pi), R * np.sin(S[0, id, 0] / Circumference * 2 * math.pi), marker = 'o', markersize = vehicleSize, markerfacecolor = '#008000', markeredgecolor = 'k')[0]
            
            else :
                if ID[id] == 0 :
                    position[id] = plt.plot(R * np.cos(S[0, id, 0] / Circumference * 2 * math.pi), R * np.sin(S[0, id, 0] / Circumference * 2 * math.pi), marker = 'o', markersize = vehicleSize, markerfacecolor = '#008000', markeredgecolor = 'k')[0]
                else :
                    position[id] = plt.plot(R * np.cos(S[0, id, 0] / Circumference * 2 * math.pi), R * np.sin(S[0, id, 0] / Circumference * 2 * math.pi), marker = 'o', markersize = vehicleSize, markerfacecolor = 'b', markeredgecolor = 'k')[0]
        #Road
        temp = np.linspace(0,2 * math.pi,100)
        plt.plot(0.87 * R * np.cos(temp), 0.87 * R * np.sin(temp),markerfacecolor = 'k', linewidth = 0.4)
        plt.plot(1.13 * R * np.cos(temp), 1.13 * R * np.sin(temp),markerfacecolor = 'k', linewidth = 0.4)
        return position

    def update(frame):   
        #for i in range (int(10/Tstep) - 1, int((TotalTime - 20) / Tstep), 10) :
        i = int(10/Tstep) - 1 + (10 * frame)
        for id in range(0,N) : 
            temp = np.linspace(0,2 * math.pi,100)
            position[id].set_xdata(R * np.cos(S[i,id,0] / Circumference * 2 * math.pi))
            position[id].set_ydata(R * np.sin(S[i,id,0] / Circumference * 2 * math.pi))

            velocityMap = [
                        (1,0,0), (1, 0.09375, 0), (1, 0.1875,0), (1, 0.28125,0), (1, 0.375,0),
                        (1, 0.46875, 0), (1, 0.5625, 0), (1, 0.65625, 0), (1, 0.75, 0), (1, 0.84375, 0), (1, 0.9375, 0),
                        (0.96875, 1, 0), (0.875, 1, 0), (0.78125, 1, 0), (0.6875, 1, 0), (0.59375, 1, 0), (0.5, 1, 0),
                        (0.40625, 1, 0), (0.3125, 1, 0), (0.21875, 1, 0), (0.125, 1, 0), (0.03125, 1, 0)
                        ]


            if S[i,id,1] < velLowerBound :
                temp = velLowerBound
            elif S[i,id,1] > velUpperBound :
                temp = velUpperBound
            else :
                temp = S[i,id,1]


            if ID[id] == 0:
                colorID = min(math.floor((temp - velLowerBound) / (velUpperBound - velLowerBound) * 22) + 1, 22) - 1
                position[id].set_markerfacecolor(velocityMap[colorID])
        fig.canvas.draw()
        return position

    ani = FuncAnimation(fig, update, frames = 7000, interval = 1, init_func=init, repeat=True, blit=True)
    plt.show()
