%% Discription
% Analyze the influence of AV number and system size on the performance.
% Use random simulations to generate data.


clc;
clear;
close all;
addpath('..\_fcn');

%% Key parameters

AV_number = 2; % 1 or 2

N_collected = 10:10:100; % The scale of system size

%% Controller Design

for iN = 1:length(N_collected)
    N = N_collected(iN);
    s_star = 20;
    gamma_s = 0.03;
    gamma_v = 0.15;
    gamma_u = 1;
    K = lqr_sdp(N,s_star,gamma_s,gamma_v,gamma_u,AV_number);
    K_collected{N} = K;
end


%% System Size Analysis

controlEnergy = zeros(length(N_collected),1);
ConvergenceTime = zeros(length(N_collected),1);
Index = zeros(length(N_collected),1);

SN = 20; % Number of random simulations

parfor iN = 1:length(N_collected)
    N=N_collected(iN);
    tic
    for iSN = 1:SN
        %% Parameters
        v_max = 30;
        acel_max = 5;
        dcel_max = -5;
        %Driver Model: OVM
        alpha = 0.6;
        beta = 0.9;
        s_st = 5;
        s_go = 35;
        
        
        %Simulation
        
        TotalTime = 300;
        Tstep = 0.01;
        NumStep = TotalTime/Tstep;
        %Scenario
        Circumference = 20*N;
        ActuationTime = 0;  %When will the controller work
        %Equilibrium
        %s_star = 0.5*(s_st+s_go);
        s_star = Circumference/N;
        v_star = v_max/2*(1-cos(pi*(s_star-s_st)/(s_go-s_st))); %THEORETICALLY equilibrium velocity
        s_des = s_star*1;
        v_des = v_max/2*(1-cos(pi*(s_des-s_st)/(s_go-s_st)));
        
        
        
        %Initial State for each vehicle
        S = zeros(NumStep,N,3);
        dev_s = 4;
        dev_v = 2;
        co_v = 1.0;
        v_ini = co_v*v_star; %Initial velocity
        %from -dev to dev
        S(1,:,1) = linspace(Circumference,s_star,N)'+(rand(N,1)*2*dev_s-dev_s);  %The vehicles are uniformly distributed on the ring road with a random deviation
        S(1,:,2) = v_ini*ones(N,1)+(rand(N,1)*2*dev_v-dev_v);
        
        % Mix or not
        mix = 1;
        controllerType = 3;
        % 1.LQR  2.FollowerStopper  3.PI saturation
        ID = zeros(1,N);
        if mix
            ID(N) = 1;
            X = zeros(2*N,NumStep);
        end
        %0. Manually Driven  1. Controller
        
        %Velocity Difference
        V_diff = zeros(NumStep,N);
        %Following Distance
        D_diff = zeros(NumStep,N);
        temp = zeros(N,1);
        %Avg Speed
        V_avg = zeros(NumStep,1);
        v_cmd = zeros(NumStep,1);  % for controller 2 & 3
        
        
        %% Controller
        
        K = K_collected{N};
        
        
        %% Simulation begins
        
        for k = 1:NumStep-1
            %Update acceleration
            temp(2:end) = S(k,1:(end-1),2);
            temp(1) = S(k,end,2);
            V_diff(k,:) = temp-reshape(S(k,:,2),N,1);
            temp(1) = S(k,end,1)+Circumference;
            temp(2:end) = S(k,1:(end-1),1);
            D_diff(k,:) = temp-reshape(S(k,:,1),N,1); %Real Following Distance
            cal_D = D_diff(k,:); %For the boundary of Optimal Veloicity Calculation
            cal_D(cal_D>s_go) = s_go;
            cal_D(cal_D<s_st) = s_st;
            %OVM Model
            %V_d = v_max/2*(1-cos(pi*(h-h_st)/(h_go-h_st)));
            %a2 = alpha*(V_h-v2)+beta*(v1-v2);
            acel = alpha*(v_max/2*(1-cos(pi*(cal_D-s_st)/(s_go-s_st)))-S(k,:,2))+beta*V_diff(k,:);
            acel(acel>acel_max)=acel_max;
            acel(acel<dcel_max)=dcel_max;
            % SD as ADAS to prevent crash
            temp(2:end) = S(k,1:(end-1),2);
            temp(1) = S(k,end,2); %temp is the velocity of the preceding vehicle
            acel_sd = (reshape(S(k,:,2).^2,N,1)-temp.^2)./2./reshape(D_diff(k,:),N,1);
            acel(acel_sd>abs(dcel_max)) = dcel_max;
            
            S(k,:,3) = acel;
            if k>ActuationTime/Tstep
                switch controllerType
                    case 1
                        X(1:2:2*N,k) = reshape(D_diff(k,:),N,1)-s_des;
                        X(2:2:2*N,k) = reshape(S(k,:,2),N,1)-v_des;
                        %X(2*N-1,k) = D_diff(k,N)-(s_ctr-(s_ctr-s_star)*N);
                        u = -K*X(:,k);
                    case 2
                        d1 = 12.5; d2=14.75; d3 = 20;
                        d = D_diff(k,N);
                        v_temp = min(S(k,N-1,2),v_star);
                        if d<=d1
                            v_cmd(k) = 0;
                        elseif d<=d2
                            v_cmd(k) = v_temp*(d-d1)/(d2-d1);
                        elseif d<=d3
                            v_cmd(k) = v_temp+(v_star-v_temp)*(d-d2)/(d3-d2);
                        else
                            v_cmd(k) = v_star;
                        end
                        u = alpha*(v_cmd(k)-S(k,N,2));
                    case 3
                        gl = 7; gu = 30; v_catch = 1; gamma_temp = 2;
                        if k-26/Tstep<=0
                            v_hisAvg = mean(S(1:k,N,2));
                        else
                            v_hisAvg = mean(S((k-26/Tstep):k,N,2));
                        end
                        v_target = v_hisAvg + v_catch*min(max((D_diff(k,N)-gl)/(gu-gl),0),1);
                        alpha_temp = min(max((D_diff(k,N)-max(2*V_diff(k,N),4))/gamma_temp,0),1);
                        beta_temp = 1-0.5*alpha_temp;
                        v_cmd(k+1) = beta_temp*(alpha_temp*v_target+(1-alpha_temp)*S(k,N-1,2))+(1-beta_temp)*v_cmd(k);
                        u = alpha*(v_cmd(k+1)-S(k,N,2));
                end
                if u>acel_max
                    u=acel_max;
                elseif u<dcel_max
                    u=dcel_max;
                end
                if (S(k,N,2)^2-S(k,N-1,2)^2)/2/(S(k,N-1,1)-S(k,N,1))>abs(dcel_max)
                    u=dcel_max;
                end
                S(k,N,3) = u;
            end
            %S(k,:,3) = S(k,:,3) + normrnd(0,0.2);
            S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
            S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
            
            
        end
        
        for k = 1:NumStep
            V_avg(k) = mean(S(k,:,2));
        end
        S(:,N,3) = smooth(S(:,N,3),50);
        
        
        controlEnergy(iN)=controlEnergy(iN)+sum(S(:,N,3).^2)*Tstep;
        k = NumStep;
        while 1
            v_minus = S(k,:,2)-V_avg(k);
            if isempty(find(v_minus>0.01,1))
                k = k-1;
            else
                break;
            end
        end
        ConvergenceTime(iN) = ConvergenceTime(iN) + k*Tstep;
        Index(iN) = Index(iN)+N;
    end
    disp(['N = ',num2str(N)]);
    toc
end
controlEnergy = controlEnergy/SN;
ConvergenceTime = ConvergenceTime/SN;
Index = Index/SN;


% switch AV_number
%     case 1
%         save(['..\_data\',date,'-InitialDeviation_SystemSize_1AV.mat']);
%     case 2
%         save(['..\_data\',date,'-InitialDeviation_SystemSize_2AV.mat']);
% end

