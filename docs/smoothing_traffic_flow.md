# Smoothing traffic flow

Although large-scale numerical simulations and small-scale experiments have shown promising results, 
a comprehensive theoretical understanding to smooth traffic flow via AVs is lacking. Here, from a 
control-theoretic perspective, we establish analytical results on the controllability, stabilizability, 
and reachability of a mixed traffic system consisting of HDVs and AVs in a ring road.

![Alt Text](images/smoothing_traffic_flow/mixed_traffic_flow_schematic diagram.png)

## Getting Started

### Matlab Implementation
#### demo_smoothing traffic_flow
Add path and initialization
```matlab
clc;
clear;
close all;
addpath('_fcn');
addpath('_data');
```

Key parameters setting. 'mix' is set to 0 by default, which means there's no AV. While 'mix = 1' means the mixed traffic scenario. 'controllerType' set the control strategies for mixed traffic control. 

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
Actuation setting for when thr controller will be triggered.
```matlab
Circumference = s_star*N;
if mix
    ActuationTime = 0; 
else
    ActuationTime = 9999;
end
```

Animation setting:
```matlab
%Animation
velUpperBound = 15; % color
velLowerBound = 8; % color
vehicleSize = 12; % MarkerSize
```

Vehicle parameters setting. v_max indicate the maximum velocity the vehicle can reach. acel_max and decel_max indicate the maximum acceleration and amximun deacceleration rate respectively. The Driver model is set by using OVM model.
```matlab
v_max = 30;
acel_max = 5;
dcel_max = -5;

%Driver Model: OVM
alpha = 0.6;
beta = 0.9;
s_st = 5;
s_go = 35;
```

Simulation setting:
``` matlab
TotalTime = 100;
Tstep = 0.01;
NumStep = TotalTime/Tstep;
```

### Python Implementation

```python
fs= 100;
[filt_b, filt_a]= butter(5, [10 14]/fs*2);
state_acquire= ACQUIRE_FCN('init', 'fs',fs);
state_filter= [];
t_start= clock;
while etime(clock, t_start) < 10*60,
  cnt_new= AQCQUIRE_FCN(state_acquire);
  [cnt_new, state_filter]= online_filt(cnt_new, state_filter, filt_b, filt.a);
  cnt= proc_appendCnt(cnt, cnt_new);
  mrk= struct('fs',cnt.fs, 'pos',size(cnt.x,1));
  epo= proc_segmentation(cnt, mrk, [-500 0]);
  fv= proc_logarithm( proc_variance( epo ));
  out= apply_separatingHyperplane(LDA, fv.x(:));
  send_xml_udp('cl_output', out);
end
```


## Animation
Two demonstrations are shown below:    

All the vehicles are HDVs: traffic wave emerges
![Alt Text](images/smoothing_traffic_flow/smoothing_traffic_flow_one_AV.gif)

There is one AV: dissipating traffic wave
![Alt Text](images/smoothing_traffic_flow/smoothing_traffic_flow_all_HDVs.gif)

## Experiment results
This folder contains files for test cases.

## Reference
Zheng, Y., Wang, J., & Li, K. (2020). Smoothing traffic flow via control 
of autonomous vehicles. IEEE Internet of Things Journal, 7(5), 3882-3896.[pdf]
