# Cooperative Formation Multiple of CAVs

Although large-scale numerical simulations and small-scale experiments have shown promising results, 
a comprehensive theoretical understanding to smooth traffic flow via AVs is lacking. Here, from a 
control-theoretic perspective, we establish analytical results on the controllability, stabilizability, 
and reachability of a mixed traffic system consisting of HDVs and AVs in a ring road.

![Alt Text](images/cooperative_formation_multiple/cooperative_formation.png)

## Getting Started

### Environment Setup

To run the code, the Modeling package, [YALMIP](https://yalmip.github.io/), and the optimization solver, [MOSEK](https://www.mosek.com/), are needed to solve the semidefinite program in controller synthesis.


YALMIP:

Please install YALMIP using the provided link. Follow the Tutorial in the YALMIP website and download the YALMIP file. Once you obtained the YALMIP folder. We can put the whole YALMIP folder into our project folder.

MOSEK:

Please follow the installation instruction in MOSEK website to install MOSEK on your machine. ** Please remember to apply a license from the MOSEK website. After obtaining the license, you can put it in your project folder and your system document MOSEK path.** The path is different depends on your system.


## Matlab Implementation

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

#### Functions

##### [ReturnObjectiveValue_ACC.m](https://github.com/soc-ucsd/mixed-traffic/blob/main/cooperative_formation_multiple_AVs/_fcn/ReturnObjectiveValue_ACC.m)



```matlab
function [ Obj, closed_loop_stability ] = ReturnObjectiveValue_SelfFeedback(AV_ID,N,alpha1,alpha2,alpha3,gammaType,kv,ks)
% Generate the system model and the optimal objective value
% A fixed Feedback gain is assumed

%% Parameter
switch gammaType
    
    case 1
        %S1
        gamma_s = 0.01;
        gamma_v = 0.05;
        gamma_u = 0.1;
        
    case 2
        %S2
        gamma_s = 0.03;
        gamma_v = 0.15;
        gamma_u = 0.1;
        
    case 3
        %S3
        gamma_s = 0.05;
        gamma_v = 0.25;
        gamma_u = 0.1;
    case 4
        gamma_s = 0.03;
        gamma_v = 0.15;
        gamma_u = 1;
    case 5
        gamma_s = 1;
        gamma_v = 1;
        gamma_u = 0;

end


%%


AV_number = length(find(AV_ID==1));

A1 = [0,-1;alpha1,-alpha2];
A2 = [0,1;0,alpha3];
C1 = [0,-1;alpha1-ks,-alpha2-kv];
C2 = [0,1;0,alpha3];


%Y = AY+Bu
A = zeros(2*N,2*N);
B = zeros(2*N,AV_number);
Q = zeros(2*N);
for i=1:N
    Q(2*i-1,2*i-1) = gamma_s;
    Q(2*i,2*i) = gamma_v;
end


A(1:2,1:2) = A1;
A(1:2,(2*N-1):2*N) = A2;
for i=2:N
    A((2*i-1):(2*i),(2*i-1):(2*i))=A1;
    A((2*i-1):(2*i),(2*i-3):(2*i-2))=A2;
end

% if alpha2^2-alpha3^2-2*alpha1>0
%     stability_condition_bool = true;
% else
%     stability_condition_bool = false;
% end
% 
% if isempty(find(real(eig(A))>0.001,1))
%     stable_bool = true;
% else
%     stable_bool = false;
% end


k=1;
for i=1:N
    if AV_ID(i) == 1
        if i == 1
            A(1:2,1:2) = C1;
            A(1:2,(2*N-1):2*N) = C2;
        else
            A((2*i-1):(2*i),(2*i-1):(2*i))=C1;
            A((2*i-1):(2*i),(2*i-3):(2*i-2))=C2;
        end
        B(2*i,k) = 1;
        k = k+1;
    end
end



if isempty(find(real(eig(A))>0.001,1))
    closed_loop_stability = true;
else
    closed_loop_stability = false;
end

%% Call Yalmip to calculate the optimum
epsilon   = 1e-8;


n = size(A,1);  % number of states
m = size(B,2);  % number of inputs

% assume each vehicle has a deviation
B1 = eye(n);
B1(1:2:n,1:2:n) = 0;
% B1 = B;
% assume disturbance is the same as inout
C = Q^0.5;


% variables
X = sdpvar(n);


% constraints
Constraints = [A*X  + (A*X )' + B1*B1' <=0,
    X - epsilon*eye(n)>=0];

obj   = trace(C*X*C');
%opts  = sdpsettings('solver','sedumi');   % call SDP solver: sedumi
opts  = sdpsettings('solver','mosek');     % call SDP solver: mosek, this is often better

sol     = optimize(Constraints,obj,opts);

Xd = value(X);


Obj = trace(C*Xd*C');

end


```

##### [ReturnObjectiveValue.m](https://github.com/soc-ucsd/mixed-traffic/blob/main/cooperative_formation_multiple_AVs/_fcn/ReturnObjectiveValue.m)



```matlab

function [ Obj,stable_bool,stability_condition_bool,K ] = ReturnObjectiveValue(AV_ID,N,alpha1,alpha2,alpha3,gammaType)
% Generate the system model and the optimal objective value


%% Parameter
switch gammaType
    
    case 1
        %S1
        gamma_s = 0.01;
        gamma_v = 0.05;
        gamma_u = 0.1;
        
    case 2
        %S2
        gamma_s = 0.03;
        gamma_v = 0.15;
        gamma_u = 0.1;
        
    case 3
        %S3
        gamma_s = 0.05;
        gamma_v = 0.25;
        gamma_u = 0.1;
    case 4
        gamma_s = 0.03;
        gamma_v = 0.15;
        gamma_u = 1;
    case 5
        gamma_s = 1;
        gamma_v = 1;
        gamma_u = 0;
        
    case 9999
        gamma_s = 0.01;
        gamma_v = 0.05;
        gamma_u = 1e-6;

end


%%


AV_number = length(find(AV_ID==1));

A1 = [0,-1;alpha1,-alpha2];
A2 = [0,1;0,alpha3];
C1 = [0,-1;0,0];
C2 = [0,1;0,0];


%Y = AY+Bu
A = zeros(2*N,2*N);
B = zeros(2*N,AV_number);
Q = zeros(2*N);
for i=1:N
    Q(2*i-1,2*i-1) = gamma_s;
    Q(2*i,2*i) = gamma_v;
end
R = gamma_u*eye(AV_number);

A(1:2,1:2) = A1;
A(1:2,(2*N-1):2*N) = A2;
for i=2:N
    A((2*i-1):(2*i),(2*i-1):(2*i))=A1;
    A((2*i-1):(2*i),(2*i-3):(2*i-2))=A2;
end

if alpha2^2-alpha3^2-2*alpha1>0
    stability_condition_bool = true;
else
    stability_condition_bool = false;
end

if isempty(find(real(eig(A))>0.001,1))
    stable_bool = true;
else
    stable_bool = false;
end


k=1;
for i=1:N
    if AV_ID(i) == 1
        if i == 1
            A(1:2,1:2) = C1;
            A(1:2,(2*N-1):2*N) = C2;
        else
            A((2*i-1):(2*i),(2*i-1):(2*i))=C1;
            A((2*i-1):(2*i),(2*i-3):(2*i-2))=C2;
        end
        B(2*i,k) = 1;
        k = k+1;
    end
end
%% Call Yalmip to calculate the optimum
epsilon   = 1e-5;


n = size(A,1);  % number of states
m = size(B,2);  % number of inputs

% assume each vehicle has a deviation
B1 = eye(n);
B1(1:2:n,1:2:n) = 0;
% B1 = B;
% assume disturbance is the same as inout

% variables
X = sdpvar(n);
Z = sdpvar(m,n);
Y = sdpvar(m);

% constraints
Constraints = [A*X - B*Z + (A*X - B*Z)' + B1*B1' <=0,
    X - epsilon*eye(n)>=0,
    [Y Z;Z' X] >=0];

obj   = trace(Q*X) + trace(R*Y);
%opts  = sdpsettings('solver','sedumi');   % call SDP solver: sedumi
opts  = sdpsettings('solver','mosek');     % call SDP solver: mosek, this is often better

sol     = optimize(Constraints,obj,opts);

Xd = value(X);
Zd = value(Z);
Yd = value(Y);


K = Zd*Xd^(-1);
Obj = trace(Q*Xd) + trace(R*Yd);

end


```



## Python Implementation

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
