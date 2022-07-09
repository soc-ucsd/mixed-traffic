##This code is adapted from MATLAB code https://github.com/wangjw18/mixed-traffic

from lqr_sdp import *
import array as arr
import math
import numpy as np
import matplotlib.pyplot as plt
import numpy.ma as M
from mpl_toolkits import mplot3d
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.colors import CSS4_COLORS
from matplotlib.animation import FuncAnimation
np.random.seed(156379)

#run this command before running mosek
#python <MSKHome>/mosek/9.2/tools/platform/osx64x86/bin/install.py

#Scenario 1 is sharp braking at 20 seconds
#Scenario 2 is random distribution of vehicles and uniform distribution of initial velocity
#Scenario 3 is Experiment B of "Controllability Analysis and Optimal Control of Mixed Traffic Flow with Human-driven and Autonomous Vehicles"
scenario = 1

#If spacing_or_velocity == 0 then display spacing graph, if 1 then display velocity graph, if 2 then display position color graph
spacing_or_velocity =  1

# Mix or not
mix = 1

# 1.Optimal Control  2.FollowerStopper  3.PI with Saturation
controllerType = 2


brakeID = 6 - 1
# In the simulation, the ID of the AV is 20, and thus the brakeID needs to minus one    

#Chose number of Vehicles
N = 20

#Choose circumference of track (0) or equilibrium spacing of vehicles s_star (1)
#If circumference is chosen then you may ignore value of spacing and same for spacing
circumference_or_spacing = 0
circumference = 400
s_star = 20    #19.36 for 14m/s v*   #20.63 for 16m/s v*   #20 for 15m/s v*      #for Experiment 7 from scenario 2 set just s_ctr to these values

if circumference_or_spacing:
    circumference = s_star*N
elif circumference_or_spacing == 0:
    s_star = circumference/N


if mix == 1 and controllerType == 1:
    gammaType = 1
    if gammaType == 1:
        gamma_s = 0.03
        gamma_v = 0.15
        gamma_u = 1
    elif gammaType == 2:
        gamma_s = 3
        gamma_v = 15
        gamma_u = 1
        
    
    K = lqr_sdp(N,s_star,gamma_s,gamma_v,gamma_u,1)
    
    #K1 = np.array( [[0.395398638473353, 0.121294783032293, 0.417338674241364, 0.0148374038356790, 0.372106495474327, -0.0647850171206889, 0.285279461640585, -0.109741329877132, 0.183960286830143, -0.121632247425548, 0.0902491529980125, -0.108977377900313, 0.0174772896793911, -0.0833003901639644, -0.0303065310092734, -0.0552939602935581, -0.0562213652242463, -0.0322391690525524, -0.0671269735091755, -0.0171916990695595, -0.0703109632991072, -0.00971899900662415, -0.0712931705515985, -0.00748227969194595, -0.0730760144740543, -0.00784843813891530, -0.0765576635371872, -0.00895277568554023, -0.0815032047914883, -0.0100245429163172, -0.0874794729270434, -0.0111184287860157, -0.0943795333512298, -0.0125332748093950, -0.102415196907486, -0.0141468211948435, -0.111608662556364, -0.0147686951380971, -0.131229687586483, 1.19230263066513]])
    #print(K-K1)
alpha_k = 0.6
    
v_max = 30
acel_max = 5
dcel_max = -5
#Driver Model: OVM
alpha = 0.6
beta = 0.9
s_st = 5
s_go = 35

if (scenario==1) or (scenario==2):
    TotalTime=100
elif (scenario==3):
    TotalTime = 700

Tstep = 0.01
NumStep = int(TotalTime/Tstep)


if mix:
    ActuationTime = 0  
else:
    ActuationTime = 9999

ActuationTimeEnd = math.inf
if (scenario==3):
    ActuationTime = 300
    ActuationTimeEnd = math.inf             #Change to 450 for experiment B of Controllability Analysis


v_star  = (v_max/2) * (1-math.cos(math.pi * (s_star - s_st)/(s_go - s_st)))
s_ctr  = s_star
v_ctr  = (v_max/2) * (1-math.cos(math.pi * (s_ctr - s_st)/(s_go - s_st)))


sd = 5 # Collision avoidance safe distance

S = np.zeros((NumStep,N,3))

#Initial state for each vehicle
if (scenario == 1) or (scenario==3): 
    dev_s = 0
    dev_v = 0
    co_v = 1.0
elif scenario == 2:      #Not necessary unless new random numbers needed
    dev_s = 7.5
    dev_v = 4
    co_v = 1.0    
    
v_ini = co_v * v_star                                                                 # What is co_v ?????

                          
var1 = np.linspace(circumference, s_star, N)
var2 = np.random.rand(N)*2*dev_s-dev_s    
S[0, :, 0] = var1 + var2
    
var1 = v_ini*np.ones([N])
var2 = (np.random.rand(N)*2*dev_v-dev_v)    
S[0, :, 1] =  var1 + var2



ID = np.zeros([N])

if mix == 1:
    ID[-1] = 1
    X = np.zeros([2*N, NumStep])


#Velocity difference
V_diff = np.zeros([NumStep,N])

#Following Distance
D_diff = np.zeros([NumStep,N])
temp = np.zeros([N])


#Average Speed
V_avg = np.zeros([NumStep,1])
v_cmd = np.zeros([NumStep,1])   #For controller's 2 and 3


##Simulation

for k in range(0,NumStep-2):
    
    #Car in front velocity
    temp[1:] = S[k,:-1,1]
    temp[0] = S[k,-1,1]
    V_diff[k,:] = temp-S[k,:,1]
    
    
    temp[0]=S[k,-1,0]+circumference
    temp[1:] = S[k,:-1,0]

    D_diff[k,:] = temp-S[k,:,0]

    cal_D = D_diff[k,:]
    cal_D[cal_D>s_go] = s_go
    cal_D[cal_D<s_st] = s_st

    
    
    #OVM Model
    
    acel2 = math.pi*(cal_D-s_st)/(s_go-s_st)
    acel1 = (1-np.cos(acel2))
    acel3 = np.zeros(N)
    
    if (scenario==3): 
        acel3 = np.random.normal(0,math.sqrt(0.5),N)
   
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
    
    
    
    if (k*Tstep>20) and (k*Tstep<23) and (scenario == 1):
        S[k,brakeID,2]=-3
        
    
    if (k>=ActuationTime/Tstep) and (k<ActuationTimeEnd/Tstep) and (mix==1):
        
        if controllerType==1:
            X[np.arange(0,2*N,2),k] = D_diff[k,:]-s_ctr
            X[np.arange(1,2*N,2),k] = S[k,:,1]-v_ctr
            u = -K@X[:,k]
            # print(u)
            # print(D_diff[k,:]-s_ctr)
            # print(S[k,:,1]-v_ctr)
            # print(X[:,k])
            
        elif controllerType==2:
            dx10 = 9.5
            dx20 = 10.75
            dx30 = 11
            
            dv_temp = min(S[k,-2,1]-S[k,-1,1],0)
            
            d1 = 1.5
            d2 = 1.0
            d3 = 0.5
            
            dx1 = dx10+dv_temp**2/2/d1
            dx2 = dx20+dv_temp**2/2/d2
            dx3 = dx30+dv_temp**2/2/d3
            
            dx = D_diff[k,N-1]
            v_temp = min(S[k,-2,1],12)
            
            
            if dx<=dx1:
                v_cmd = 0
            elif dx<=dx2:                                                        #??????? d2 is greater than d3
                v_cmd = v_temp*(dx-dx1)/(dx2-dx1)
            elif dx<=dx3:
                v_cmd = v_temp+(v_ctr-v_temp)*(dx-dx2)/(dx3-dx2)
            else:
                v_cmd = v_ctr
            
            u = alpha_k*(v_cmd-S[k,-1,1])
            
        elif controllerType==3:
            gl = 7
            gu = 30
            v_catch = 1
            gamma_temp = 2
            
            if k-38/Tstep<=ActuationTime/Tstep:
                v_hisAvg = np.mean(S[int(ActuationTime/Tstep)-1:k,-1,1])
            else:
                v_hisAvg = np.mean(S[int((k-38/Tstep)-ActuationTime/Tstep)-1:k,-1,1])
            
            v_target = v_hisAvg + v_catch*min(max((D_diff[k,-1]-gl)/(gu-gl),0),1)
            alpha_temp = min(max((D_diff[k,-1]-max(2*V_diff[k,-1],4))/gamma_temp, 0),1)
            beta_temp = 1-0.5*alpha_temp
            v_cmd[k+1] = beta_temp*(alpha_temp*v_target+(1-alpha_temp)*S[k,-2,1])+(1-beta_temp)*v_cmd[k]
            u = alpha_k*(v_cmd[k+1]-S[k,-1,1])
            
        
        if u>acel_max:
            u=acel_max
        elif u<dcel_max:
            u=dcel_max
        
        
        if ( ( (S[k,-1,1]**2-S[k,-2,1]**2) /2 ) / (S[k,-2,0]-S[k,-1,0]-sd) )>abs(dcel_max):
            u=dcel_max
            
        # print(S[k,-1,1]**2-S[k,-2,1]**2)
        # print((S[k,-2,0]-S[k,-1,0]-sd))
        # print( ( (S[k,-1,1]**2-S[k,-2,1]**2) /2 ) / (S[k,-2,0]-S[k,-1,0]-sd) )            
        # print(S[k,-2,0])
        # print(S[k,-1,0])
        # print(u)
        S[k,-1,2] = u
 
        
    S[k+1,:,1] = S[k,:,1] + Tstep*S[k,:,2]
    S[k+1,:,0] = S[k,:,0] + Tstep*S[k,:,1]
    


for k in range(NumStep):
    V_avg[k] = np.mean(S[k,:,1])

    
        
        
#Settling Time
final_velocity = V_avg[NumStep-2]
above_2_percent = final_velocity*1.03
below_2_percent = final_velocity*0.97

settling_time = 0
for k in range(NumStep-2,0,-1):
    for j in range(N):
        if (S[k,j,1] > above_2_percent) or (S[k,j,1] < below_2_percent):
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
 



#Display data

fig = plt.figure()
x = np.arange(0,NumStep)

# syntax for 3-D projection


#y = np.linspace(0,20,20)

if spacing_or_velocity==0:
    ax = plt.axes(projection ='3d')
    for i in range(N):
        z = np.ones(NumStep-1)*i
        
        if i==N-1 and mix==1:
            ax.plot3D(z, x[:-1], S[:-1,i-1,0] - S[:-1,i,0], 'red', linewidth=0.5)
            continue

        if i==0:
            ax.plot3D(z, x[:-1], S[:-1,i-1,0] - S[:-1,i,0] + circumference, 'blue', linewidth=0.5)
            continue  
        
        ax.plot3D(z, x[:-1], S[:-1,i-1,0] - S[:-1,i,0], 'blue', linewidth=0.5)
             
    
    ax.set_yticks(np.linspace(0,NumStep,5))
    ax.set_yticklabels(np.linspace(0,TotalTime,5))
    ax.set_xlabel("Vehicle ID")
    ax.set_ylabel("Time")
    ax.set_zlabel("Spacing from vehicle ahead")
    title = "N=" + str(N) + ", mix=" + str(mix) +", controller=" + str(controllerType)
    ax.set_title(title)
    plt.show()
    
if spacing_or_velocity==1:
    ax = plt.axes(projection ='3d')
    for i in range(N):
        z = np.ones(NumStep-1)*i
        
        if i==N-1 and mix==1:
            ax.plot3D(z, x[:-1], S[:-1,i,1], 'red', linewidth=0.5)
            continue
        
        ax.plot3D(z, x[:-1], S[:-1,i,1], 'blue', linewidth=0.5)
             
    
    ax.set_yticks(np.linspace(0,NumStep,5))
    ax.set_yticklabels(np.linspace(0,TotalTime,5))
    ax.set_xlabel("Vehicle ID")
    ax.set_ylabel("Time")
    ax.set_zlabel("Vehicle Velocity")
    title = "N=" + str(N) + ", mix=" + str(mix) +", controller=" + str(controllerType)
    ax.set_title(title)
    plt.show()    
        
        
        
if spacing_or_velocity==2:
    
    for i in range(N):
        y = S[:,i,0]%circumference
        y[y>399]=np.nan
        
        
        #y = M.array(y)
        #masked_y = M.masked_where(y>399,y)
        z = S[:,i,1]
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)   
        
        
        lc = LineCollection(segments, array=z, cmap=LinearSegmentedColormap.from_list('rg',["black","r", "orange", "y", "limegreen"], N=256), norm=plt.Normalize(0,25), linewidth=0.5)#, alpha=alpha)
            
        ax = plt.gca()
        ax.add_collection(lc)
        
        #colorline(x, S[:,i,0]%circumference, S[:,i,1])
        #plt.scatter(x=x, y=S[:,i,0]%circumference, c=S[:,i,1], s=0.00001, cmap=LinearSegmentedColormap.from_list('rg',["b","r","y","g"], N=256), norm=plt.Normalize(0,15))
        #plt.plot(x, S[:,i,0]%circumference, c=S[:,i,1])
        
    plt.xlim(20000, 60000)
    plt.ylim(0,400)
    plt.show()
    

#plt.plot(S[:,-1,2])
#plt.show()

# Animation
velUpperBound = 15 # color
velLowerBound = 8 # color
vehicleSize = 10.5 # MarkerSize

fig = plt.figure(figsize=(16, 9), dpi=80) # ADD A PARAMETER TO NUMERICALLY IDENTIFY FIGURE
fig.set_facecolor('w') 
axs1 = plt.subplot(121)
axs1.set_aspect('equal')
plt.axis('off')

R = circumference/2/math.pi * 2
position = [None] * N

def init():
    #Vehicles
    for id in range(N) :
        if not mix :
            position[id] = plt.plot(R * np.cos(S[0, id, 0] / circumference * 2 * math.pi), R * np.sin(S[0, id, 0] / circumference * 2 * math.pi), marker = 'o', markersize = vehicleSize, markerfacecolor = '#008000', markeredgecolor = 'k')[0]
        
        else :
            if ID[id] == 0 :
                position[id] = plt.plot(R * np.cos(S[0, id, 0] / circumference * 2 * math.pi), R * np.sin(S[0, id, 0] / circumference * 2 * math.pi), marker = 'o', markersize = vehicleSize, markerfacecolor = '#008000', markeredgecolor = 'k')[0]
            else :
                position[id] = plt.plot(R * np.cos(S[0, id, 0] / circumference * 2 * math.pi), R * np.sin(S[0, id, 0] / circumference * 2 * math.pi), marker = 'o', markersize = vehicleSize, markerfacecolor = 'b', markeredgecolor = 'k')[0]
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
        position[id].set_xdata(R * np.cos(S[i,id,0] / circumference * 2 * math.pi))
        position[id].set_ydata(R * np.sin(S[i,id,0] / circumference * 2 * math.pi))

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

ani = FuncAnimation(fig, update, frames = 7000, interval = 5, init_func=init, repeat=True, blit=True)
plt.show()      
