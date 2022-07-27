# Cooperative Formation Multiple

Although large-scale numerical simulations and small-scale experiments have shown promising results, 
a comprehensive theoretical understanding to smooth traffic flow via AVs is lacking. Here, from a 
control-theoretic perspective, we establish analytical results on the controllability, stabilizability, 
and reachability of a mixed traffic system consisting of HDVs and AVs in a ring road.

![Alt Text](images/cooperative_formation_multiple/cooperative_formation.png)

## Getting Started

### Matlab Implementation

#### Main File

##### [demo_cooperative_formation.m](https://github.com/soc-ucsd/mixed-traffic/blob/main/cooperative_formation_multiple_AVs/demo_cooperative_formation.m)
Add path and initialization
```matlab
clc;
clear;
close all;

addpath('_fcn');
addpath('_data');

```

Key parameters setting. N indicate the total number of vehicle in the experiment. brake_ID means the Position of the perturbation. 
platoon_bool indicate that if the Autonomous vehicles formation. (If they are lined up in platoon.)


```matlab
N = 20;
AV_number = 0; % 0 or 1 or 2 or 4
platoon_bool= 0;
brakeID = 15;

```

Parameter Setting.
``` matlab
if AV_number == 0
    mix = 0;
    ActuationTime = 9999;
else
    mix = 1;
end

ID = zeros(1,N); %0. Manually Driven  1. Controller
if mix
    ActuationTime = 0;
    % Define the spatial formation of the AVs
    switch AV_number
        case 4
            if platoon_bool
                ID(9) = 1;
                ID(10) = 1;
                ID(11) = 1;
                ID(12) = 1;
            else
                ID(3) = 1;
                ID(8) = 1;
                ID(13) = 1;
                ID(18) = 1;
            end
        case 2
            if platoon_bool
                ID(10) = 1;
                ID(11) = 1;
                
            else
                ID(5) = 1;
                ID(15) = 1;
            end
        case 1
            ID(20) = 1;
    end
    
end

```

Controller parameters setting:
```matlab
gammaType = 2;

v_star = 15;
```


OVM parameter setting:
```matlab
s_star = 20;
v_max  = 30;
s_st   = 5;
s_go   = 35;
```
There are two type of parameters setting for OVM model in our experiments. Please use one type of setting 
when running the simulation.

- Type 1 alpha, beta seting is for describing the carfollowing behavior of HDVs.
- Type 2 is another type of model parameter setting.

```matlab
%%%%% Type1 %%%%%%%
alpha  = 0.6;
beta   = 0.9;
%%%%%%%%% Type2 %%%%%%%%%
alpha  = 1.0;
beta   = 1.5;
```







### Python Implementation

#### Main File
 

    mkdocs.yml    # The configuration file.
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files.
* `mkdocs new [dir-name]` - Create a new project.
* `mkdocs serve` - Start the live-reloading docs server.
* `mkdocs build` - Build the documentation site.
* `mkdocs -h` - Print help message and exit.

##### temp

#### Functions

## Experiment results
Two demonstrations are shown below:    

Uniform Distribution:
![Alt Text](images/cooperative_formation_multiple/platoon_formation.gif)

Platoon Formation:
![Alt Text](images/cooperative_formation_multiple/uniform_distribution.gif)

This folder contains files for test cases.

## Reference

- Li, K., Wang, J., & Zheng, Y. (2020). Cooperative Formation of Autonomous Vehicles in Mixed Traffic Flow: Beyond Platooning. *arXiv preprint arXiv:2009.04254*.[[pdf](https://wangjw18.github.io/files/2020-arXiv.pdf)]
- Li, K., Wang, J., & Zheng, Y. (2020). Optimal Formation of Autonomous Vehicles in Mixed Traffic Flow. *In 21st IFAC World Congress*. [[pdf](https://wangjw18.github.io/files/2020-IFAC.pdf)] [[slides](https://wangjw18.github.io/files/2020-IFAC-slides.pdf)]