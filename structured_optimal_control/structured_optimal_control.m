%% Description
% Design a structured optimal controller under communication constraint
% Correspond to Section IV in the following paper
% Controllability Analysis and Optimal Control of Mixed Traffic Flow With Human-Driven and Autonomous Vehicles

clc;
clear;
close all;

addpath('_fcn');

%% Key parameters

comm_limited = 1; % whether there is communication constraint
CR = 5; % communication range if there is communication constraint


%% Parameters

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

%Cost Function Weight
gamma_s = 0.03;
gamma_v = 0.15;
gamma_u = 1;

%% Controller design


[A,B1,B2,Q,R] = system_model(N,AV_number,alpha,beta,v_max,s_st,s_go,s_star,gamma_s,gamma_v,gamma_u);

if comm_limited
    K_Pattern = pattern_generation(N,AV_number,CR);
    [K,Info] = optsi(A,B1,B2,K_Pattern,Q,R);
else
    K = lqrsdp(A,B1,B2,Q,R);
end

% save(['_data\',date,'_setup_and_controller']);