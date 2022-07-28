# Smoothing Traffic Flow

Although large-scale numerical simulations and small-scale experiments have shown promising results, 
a comprehensive theoretical understanding to smooth traffic flow via AVs is lacking. Here, from a 
control-theoretic perspective, we establish analytical results on the controllability, stabilizability, 
and reachability of a mixed traffic system consisting of HDVs and AVs in a ring road.

![Alt Text](images/smoothing_traffic_flow/mixed_traffic_flow_schematic diagram.png)

## Getting Started

### Matlab Implementation

#### Main File

##### [demo_smoothing traffic_flow.m](https://github.com/soc-ucsd/mixed-traffic/blob/main/smoothing_traffic_flow_via_AVs/demo_smoothing_traffic_flow.m)
Add path and initialization
```matlab
clc;
clear;
close all;
addpath('_fcn');
addpath('_data');
```

Key parameters setting:

 'mix' is set to 0 by default, which means there's no AV. While 'mix = 1' means the mixed traffic scenario. 'controllerType' set the control strategies for mixed traffic control. 

'controllerType' value to strategies:

- 1.Optimal Control  
- 2.FollowerStopper 
- 3.PI with Saturation

```matlab
mix = 0;
controllerType = 1;
```

brakeID indicate the index of vehicle that will brake first (interuption happen). N indicate the total number of vehicle in the experiment. s_star means the  equilibrium spacing of each vehicle.

```matlab
brakeID = 6 - 1;
N = 20;
s_star = 20;
```
ControllerType parameter setting:

- gamma_s means the preference setting of vehicle spacing.
- gamma_v means the preference setting of vehicle velocity.
- gamma_u means the preference setting of vehicle input.

```matlab
if mix && controllerType == 1
    %Cost Function Weight
    gammaType = 1;
    switch gammaType
        case 1
            gamma_s = 0.03;
            gamma_v = 0.15;
            gamma_u = 1;
            
        case 2
            gamma_s = 3;
            gamma_v = 15;
            gamma_u = 1;
    end
    K = lqr_sdp(N,s_star,gamma_s,gamma_v,gamma_u,1);
end
```

alpha_k setting is only for FollowerStopper strategy.
```matlab
alpha_k = 0.6;
```
Animation setting:

- velUpperBound means the upper bound of vehicle speed. It was set to 15 m/s. Corresponding to the green color for Animation. 
- velLowerBound indicate the lower bound of vehicle speend. It was set to 8 m/s. Corresponding to the red color for Animation.
- vehicle size was set to 12 by default. 
```matlab
velUpperBound = 15; % color
velLowerBound = 8; % color
vehicleSize = 12; % MarkerSize
```

Vehicle parameters setting:

- v_max indicate the maximum velocity the vehicle can reach. acel_max and decel_max indicate the maximum acceleration and amximun deacceleration rate respectively. The Driver model is set by using OVM model.
- acel_max and dcel_max indicate the maximum acceleration and mininmum acceleration.
```matlab
v_max = 30;
acel_max = 5;
dcel_max = -5;
```

OVM Model setting:

- alpha and beta was set to 0.6 and 0.9. Where alpha > 0 reflects the driver’s sensitivity to the difference between the current velocity and the spacing-dependent
desired velocity V (si(t)), and beta > 0 reflects the driver’s
sensitivity to the difference between the velocities of the front vehicle and the preceding vehicle

```matlab
alpha = 0.6;
beta = 0.9;
s_st = 5;
s_go = 35;
```

Simulation setting:

```matlab
TotalTime = 100;
Tstep = 0.01;
NumStep = TotalTime/Tstep;
```

The actuation setting set the time when the controller will be engaged.
```matlab
Circumference = s_star*N;
if mix
    ActuationTime = 0;  %When will the controller work
else
    ActuationTime = 9999;
end
```

Equilibrium state setting.
```matlab
s_star = Circumference/N;
v_star = v_max/2*(1-cos(pi*(s_star-s_st)/(s_go-s_st))); %THEORETICALLY equilibrium velocity
s_ctr = s_star*1.0;
v_ctr = v_max/2*(1-cos(pi*(s_ctr-s_st)/(s_go-s_st)));
```

Safe distance setting. The minimum safe distance value is set to zero since the vehicle length is ignored.
```matlab
sd = 0;
```

Initial State for each vehicle
```matlab
S = zeros(NumStep,N,3);
dev_s = 0;
dev_v = 0;
co_v = 1.0;
v_ini = co_v*v_star; %Initial velocity
```
#### Functions


### Python Implementation
The Python implementation follows three main scenarios - 

Scenario 1 is sharp braking at 20 seconds.

Scenario 2 is random distribution of vehicles and uniform distribution of initial velocity.

Scenario 3 is Experiment B of "Controllability Analysis and Optimal Control of Mixed Traffic Flow with Human-driven and Autonomous Vehicles".

The implementation follows the MATLAB version closely, utilizing NumPy instead of MATLAB methods where appropriate. 

#### Functions
There are two main functions in the Python implementation :

1. lqr_sdp
```python
W = cp.Variable((m+n,m+n), symmetric=True)
    #                W[0:m,0:m] == Y,
    #                W[m:,0:m] == Z,
    #                W[m:,m:] == X,
    
    objective    = cp.Minimize( cp.trace( Q@W[m:,m:] ) + cp.trace( R@W[0:m,0:m] ) )
    constraints  = [(A@W[m:,m:] - B@W[0:m,m:]) + (A@W[m:,m:] - B@W[0:m,m:]).T + H@(H).T << 0]
    constraints += [ W[m:,m:] - epsilon*np.identity(n) >> 0]
    constraints += [W >> 0]

```
This code sets up our optimization problem, generating a


## Experiment results
Two demonstrations are shown below:    

All the vehicles are HDVs: traffic wave emerges
![Alt Text](images/smoothing_traffic_flow/smoothing_traffic_flow_one_AV.gif)

There is one AV: dissipating traffic wave
![Alt Text](images/smoothing_traffic_flow/smoothing_traffic_flow_all_HDVs.gif)

This folder contains files for test cases.

## Reference

- Zheng, Y., Wang, J., & Li, K. (2020). Smoothing traffic flow via control of autonomous vehicles. *IEEE Internet of Things Journal*, *7*(5), 3882-3896.[[pdf](https://wangjw18.github.io/files/2018-arXiv.pdf)]