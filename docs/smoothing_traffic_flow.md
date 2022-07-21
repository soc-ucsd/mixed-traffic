# Smoothing traffic flow

Although large-scale numerical simulations and small-scale experiments have shown promising results, 
a comprehensive theoretical understanding to smooth traffic flow via AVs is lacking. Here, from a 
control-theoretic perspective, we establish analytical results on the controllability, stabilizability, 
and reachability of a mixed traffic system consisting of HDVs and AVs in a ring road.

![Alt Text](images/smoothing_traffic_flow/mixed_traffic_flow_schematic diagram.png)

## Getting Started

### Matlab Implementation
#### demo_smoothing traffic_flow
Add path
```matlab
clc;
clear;
close all;
addpath('_fcn');
addpath('_data');
```

Key parameters setting. 'mix' is set to 0 by default, which means there's no AV. 'mix' means the mixed traffic scenario. 'controllerType' set the control strategies for mixed traffic control. 'controllerType' value to strategies:
-1.Optimal Control  
-2.FollowerStopper 
-3.PI with Saturation

```matlab
% Mix or not
mix = 0;
% 1.Optimal Control  2.FollowerStopper  3.PI with Saturation
controllerType = 1;
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
