# Structured optimal control

Due to the limit of communication abilities in practice, 
the CAV can only receive partial information of the global 
traffic system for its feedback. Therefore, it is important to consider the local 
available information of the neighboring vehicles. 
This leads to the notion of structured controller design.

![Alt Text](images/structured_optimal_control/structured_control.png)

## Getting Started

### Matlab Implementation
#### structured_optimal_control.m
Add path and initialization
```matlab
clc;
clear;
close all;
addpath('_fcn');
```

Key parameters setting. The comm_limited indicated that whether there is communication constraint. While the CR indicate the communication range if there is communication constraint.
```matlab
comm_limited = 1;
CR = 5; 
```

Experiment parameter setting. N indicate the total number of vehicle in the experiment.

```matlab
N = 20;
V_star = 15;

alpha  = 0.6+0.1-0.2*rand(N,1);
beta   = 0.9+0.1-0.2*rand(N,1);
v_max  = 30*ones(N,1);
s_st   = 5*ones(N,1);
s_go   = 30+10*rand(N,1);
v_star = V_star*ones(N,1);
s_star = acos(1-v_star./v_max*2)/pi.*(s_go-s_st)+s_st;

AV_number = 1;
```

Cost Function Weight Setting.

- gamma_s means the preference setting of vehicle spacing.
- gamma_v means the preference setting of vehicle velocity.
- gamma_u means the preference setting of vehicle input.

```matlab
gamma_s = 0.03;
gamma_v = 0.15;
gamma_u = 1;
```

Controller design.

```matlab
[A,B1,B2,Q,R] = system_model(N,AV_number,alpha,beta,v_max,s_st,s_go,s_star,gamma_s,gamma_v,gamma_u);

if comm_limited
    K_Pattern = pattern_generation(N,AV_number,CR);
    [K,Info] = optsi(A,B1,B2,K_Pattern,Q,R);
else
    K = lqrsdp(A,B1,B2,Q,R);
end

```


### Python Implementation

    mkdocs.yml    # The configuration file.
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files.
* `mkdocs new [dir-name]` - Create a new project.
* `mkdocs serve` - Start the live-reloading docs server.
* `mkdocs build` - Build the documentation site.
* `mkdocs -h` - Print help message and exit.

## Animation
Two demonstrations are shown below:    

All the vehicles are HDVs: traffic wave emerges
![Alt Text](images/smoothing_traffic_flow/smoothing_traffic_flow_one_AV.gif)

There is one AV: dissipating traffic wave
![Alt Text](images/smoothing_traffic_flow/smoothing_traffic_flow_all_HDVs.gif)

## Experiment results
This folder contains files for test cases.

## Reference
- Wang, J., Zheng, Y., Xu, Q., Wang, J., & Li, K. (2020). Controllability Analysis and Optimal Control of Mixed Traffic Flow with Human-driven and Autonomous Vehicles. IEEE Transactions on Intelligent Transportation Systems, 1-15.[[pdf](https://wangjw18.github.io/files/2020-arXiv.pdf)]
- Wang, J., Zheng, Y., Xu, Q., Wang, J., & Li, K. (2019, June). Controllability analysis and optimal controller synthesis of mixed traffic systems. 
In 2019 IEEE Intelligent Vehicles Symposium (IV) (pp. 1041-1047). IEEE. [[pdf](https://wangjw18.github.io/files/2020-IFAC.pdf)] [[slides](https://wangjw18.github.io/files/2020-IFAC-slides.pdf)]
