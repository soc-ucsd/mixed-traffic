# Structured Optimal Control

Due to the limit of communication abilities in practice, 
the CAV can only receive partial information of the global 
traffic system for its feedback. Therefore, it is important to consider the local 
available information of the neighboring vehicles. 
This leads to the notion of structured controller design.

![Alt Text](images/structured_optimal_control/structured_control.png)

## Getting Started

### Environment Setup

To run the code, the Modeling package, [YALMIP](https://yalmip.github.io/), and the optimization solver, [MOSEK](https://www.mosek.com/), are needed to solve the semidefinite program in controller synthesis.


YALMIP:

Please install YALMIP using the provided link. Follow the Tutorial in the YALMIP website and download the YALMIP file. Once you obtained the YALMIP folder. We can put the whole YALMIP folder into our project folder.

MOSEK:

Please follow the installation instruction in MOSEK website to install MOSEK on your machine. ** Please remember to apply a license from the MOSEK website. After obtaining the license, you can put it in your project folder and your system document MOSEK path.** The path is different depends on your system.


## Matlab Implementation

#### Main File

##### [structured_optimal_control.m](https://github.com/soc-ucsd/mixed-traffic/blob/main/structured_optimal_control/structured_optimal_control.m)
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

Experiment parameter setting: 

N indicate the total number of vehicle in the experiment. V_star indicate the equilibrium velocity.

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

Cost Function Weight Setting:

- gamma_s means the preference setting of vehicle spacing.
- gamma_v means the preference setting of vehicle velocity.
- gamma_u means the preference setting of vehicle input.

```matlab
gamma_s = 0.03;
gamma_v = 0.15;
gamma_u = 1;
```

Controller design:

```matlab
[A,B1,B2,Q,R] = system_model(N,AV_number,alpha,beta,v_max,s_st,s_go,s_star,gamma_s,gamma_v,gamma_u);

if comm_limited
    K_Pattern = pattern_generation(N,AV_number,CR);
    [K,Info] = optsi(A,B1,B2,K_Pattern,Q,R);
else
    K = lqrsdp(A,B1,B2,Q,R);
end

```

#### Functions

##### [system_model.m](https://github.com/soc-ucsd/mixed-traffic/blob/main/structured_optimal_control/_fcn/system_model.m)

This function take the a series of parameters setting as input to generate the system model paramters (A,B1,B2,Q,R).

- Q is the Kalman controllability matrix
- A and B is the matrix paramters for the linearized state-space model for the mixed traffic system x˙(t) = Ax(t) + Bu(t)
- R is calculated by gamma_u*eye(AV_number,AV_number) Both Q and R consist of the system-level performance output z. Indicating that we allow the perturbation to arise from anywhere in the traffic flow, and the performance output z(t) takes into account all the vehicles’ deviations in the traffic flow. This
setup indicates a system-level consideration.



```matlab
function [A,B1,B2,Q,R] = system_model(N,AV_number,alpha,beta,v_max,s_st,s_go,s_star,gamma_s,gamma_v,gamma_u)


alpha1 = alpha.*v_max/2*pi./(s_go-s_st).*sin(pi*(s_star-s_st)./(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;

C1 = [0,-1;0,0];
C2 = [0,1;0,0];


pos1 = 1;
pos2 = N;

%Y = AY+Bu
A = zeros(2*N,2*N);

for i=1:(N-1)
    A((2*i-1):(2*i),(2*pos1-1):(2*pos1))=[0,-1;alpha1(i),-alpha2(i)];
    A((2*i-1):(2*i),(2*pos2-1):(2*pos2))=[0,1;0,alpha3(i)];
    pos1 = pos1+1;
    pos2 = mod(pos2+1,N);
end
A((2*N-1):(2*N),(2*pos1-1):(2*pos1))=C1;
A((2*N-1):(2*N),(2*pos2-1):(2*pos2))=C2;
%Controller

Q = zeros(2*N);
for i=1:N
    Q(2*i-1,2*i-1) = gamma_s;
    Q(2*i,2*i) = gamma_v;
end

B2 = zeros(2*N,AV_number);
B2(2*N,AV_number) = 1;
if AV_number == 2
    AV2_Index = floor(N/2);
    A((2*AV2_Index-1):(2*AV2_Index),(2*AV2_Index-1):(2*AV2_Index))=C1;
    A((2*AV2_Index-1):(2*AV2_Index),(2*AV2_Index-3):(2*AV2_Index-2))=C2;
    B2(2*AV2_Index,1) = 1;
end

B1 = zeros(2*N,N);
for i=1:N
    B1(2*i,i) = 1;
end

R = gamma_u*eye(AV_number,AV_number);


end

```


##### [pattern_generation.m](https://github.com/soc-ucsd/mixed-traffic/blob/main/structured_optimal_control/_fcn/pattern_generation.m)



```matlab
function [ K_Pattern ] = pattern_generation( N,AV_number,CR )
switch AV_number
    case 1
        K_Pattern = zeros(1,2*N);
        for i = 1:CR
            K_Pattern(1,2*i-1:2*i) = [1,1];
        end
        for i = N-CR : N-1
            K_Pattern(1,2*i-1:2*i) = [1,1];
        end
        K_Pattern(1,2*N-1:2*N) = [1,1];
    case 2
        if CR>=N-floor(N/2)
            K_Pattern = ones(2,2*N);
        else
            K_Pattern = zeros(2,2*N);
            % row 1
            for i = floor(N/2)-CR : floor(N/2)+CR
                K_Pattern(1,2*i-1:2*i) = [1,1];
            end
            % row 2
            
            for i = 1:CR
                K_Pattern(2,2*i-1:2*i) = [1,1];
            end
            for i = N-CR : N-1
                K_Pattern(2,2*i-1:2*i) = [1,1];
            end
            K_Pattern(2,2*N-1:2*N) = [1,1];
        end
        
end

```

##### [lqrsdp.m](https://github.com/soc-ucsd/mixed-traffic/blob/main/structured_optimal_control/_fcn/lqrsdp.m)

This function will generate the optimal controller strategy's K value (the feedback gain) for the control input u(t) setup.

Optimal control input: u(t) = −Kx

```matlab
function [K_Opt,Info] = optsi(A,B1,B2,K_Pattern,Q,R)
% For a given pattern of K, calculate the optimal feedback gain using
% sparsity invirance
% A: system matrix;  B1: distrubance matrix;  B2: control input matrix;

n = size(A,1);
m = size(B2,2); % number of driver nodes
epsilon = 1e-5;

%% Sparsity invariance
Tp = K_Pattern;
Rp = pattern_invariance(Tp);

%% variables
X = sdpvar(n);               %% block diagonal X
for i = 1:n
    for j = i:n
       if Rp(i,j) == 0
           X(i,j) = 0; X(j,i) = 0;
       end
    end
end

Z = sdpvar(m,n);        %% Matrix Z has sparsity pattern in K_Pattern
for i = 1:m
    for j = 1:n
       if Tp(i,j) == 0
           Z(i,j) = 0;
       end
    end
end
Y = sdpvar(m);
%% constraint

Const = [X-epsilon*eye(n) >= 0, [Y Z; Z' X] >= 0,... 
    (A*X-B2*Z)+(A*X-B2*Z)'+B1*B1' <= 0];

%% cost function

Obj = trace(Q*X)+trace(R*Y);

%% solution via mosek
ops = sdpsettings('solver','mosek');
Info = optimize(Const,Obj,ops);
X1 = value(X);
Z1 = value(Z);
K_Opt = Z1*X1^(-1);


end

```

##### [pattern_invariance.m](https://github.com/soc-ucsd/mixed-traffic/blob/main/structured_optimal_control/_fcn/pattern_invariance.m)

This function will generate a maximally sparsity-wise invariant (MSI) subplace with respect to X. More detailed can be refered to paper "On Separable Quadratic Lyapunov Functions for Convex Design of Distributed Controllers".

```matlab
function [ X ] = pattern_invariance( S )
% Generate a maximally sparsity-wise invariant (MSI) subplace with respect to X
% See Section IV of the following paper
% "On Separable Quadratic Lyapunov Functions for Convex Design of Distributed Controllers"

m = size(S,1);
n = size(S,2);
X = ones(n,n);

% Analytical solution with complexity mn^2
for i = 1:m
        for k = 1:n
                if S(i,k)==0
                        for j = 1:n
                                if S(i,j) == 1
                                        X(j,k) = 0;
                                end
                        end
                end
        end
end

% symmetric part
Xu = triu(X').*triu(X);
X  = Xu + Xu';
X  = full(spones(X));

end

```
##### [optsi.m](https://github.com/soc-ucsd/mixed-traffic/tree/main/structured_optimal_control/_fcn)

For a given pattern of K, calculate the optimal feedback gain using sparsity invirance. Where A indicate the system matrix, B1 indicate the distrubance matrix and B2 indicate the control input matrix.

```matlab
function [K_Opt,Info] = optsi(A,B1,B2,K_Pattern,Q,R)
% For a given pattern of K, calculate the optimal feedback gain using
% sparsity invirance
% A: system matrix;  B1: distrubance matrix;  B2: control input matrix;

n = size(A,1);
m = size(B2,2); % number of driver nodes
epsilon = 1e-5;

%% Sparsity invariance
Tp = K_Pattern;
Rp = pattern_invariance(Tp);

%% variables
X = sdpvar(n);               %% block diagonal X
for i = 1:n
    for j = i:n
       if Rp(i,j) == 0
           X(i,j) = 0; X(j,i) = 0;
       end
    end
end

Z = sdpvar(m,n);        %% Matrix Z has sparsity pattern in K_Pattern
for i = 1:m
    for j = 1:n
       if Tp(i,j) == 0
           Z(i,j) = 0;
       end
    end
end
Y = sdpvar(m);
%% constraint

Const = [X-epsilon*eye(n) >= 0, [Y Z; Z' X] >= 0,... 
    (A*X-B2*Z)+(A*X-B2*Z)'+B1*B1' <= 0];

%% cost function

Obj = trace(Q*X)+trace(R*Y);

%% solution via mosek
ops = sdpsettings('solver','mosek');
Info = optimize(Const,Obj,ops);
X1 = value(X);
Z1 = value(Z);
K_Opt = Z1*X1^(-1);


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

#### Functions



## Experiment Results
### Experiment A
The numerical experiments tested the proposed controller alongside whether or not the proposed condition (equation 28 in the reference paper) for CAV spacing was satisfied. For this, a traffic system of 19 HDVs and 1 CAV was tested on a ring-road setup. Resulting velocity of each vehicle was graphed.

![Alt Text](images/structured_optimal_control/Structured-optimal-results.png)
The above figure is the velocity profile of each vehicle (Experiment A). (a) All the vehicles are HDVs. (b)—(f) One vehicle is CAV with the proposed method. The desired equilibrium velocity 'v*' is 15 m/s, 16 m/s, 14 m/s in (b), (c)(e), (d)(f), respectively. In (b)(c)(d) the value of s<sub>1</sub>* is determined according to the spacing condition, whereas in (e)(f) the spacing condition is not satisfied.

### Experiment B
Experiment B is conducted to test the controller’s ability to dissipate stop-and-go waves. A random noise following the normal distribution, N (0, 0.2), is added to the acceleration signal of each vehicle. This corresponds to the traffic situations where small perturbations are generated naturally inside the traffic flow.


## Reference
- Wang, J., Zheng, Y., Xu, Q., Wang, J., & Li, K. (2020). Controllability Analysis and Optimal Control of Mixed Traffic Flow with Human-driven and Autonomous Vehicles. IEEE Transactions on Intelligent Transportation Systems, 1-15.[[pdf](https://wangjw18.github.io/files/2020-arXiv.pdf)]
- Wang, J., Zheng, Y., Xu, Q., Wang, J., & Li, K. (2019, June). Controllability analysis and optimal controller synthesis of mixed traffic systems. 
In 2019 IEEE Intelligent Vehicles Symposium (IV) (pp. 1041-1047). IEEE. [[pdf](https://wangjw18.github.io/files/2020-IFAC.pdf)] [[slides](https://wangjw18.github.io/files/2020-IFAC-slides.pdf)]
