%% Discription
% Heterogeneous setup for HDVs
% Each vehicle has an initial deviation
% Correspond to Fig. 7(a)-(d) in the following paper
% Controllability Analysis and Optimal Control of Mixed Traffic Flow With Human-Driven and Autonomous Vehicles
% Pay attention to the design of the variable 's_ctr' to find out why it...
% succeeds to reach the target velocity (see Section IV.C in the paper)

clc;
clear;
close all;

addpath('..\_data');
addpath('..\_fcn');

%% Key parameters

% mix = 0; --> Fig. 7(a)
% mix = 1; v_ctr_dev = 0; --> Fig. 7(b)
% mix = 1; v_ctr_dev = 1; --> Fig. 7(c)
% mix = 1; v_ctr_dev = -1; --> Fig. 7(d)

% Mix or not
mix = 1;
% Deviation of the target velocity when there is AV control
v_ctr_dev = 0; 

%% Parameters

v_ctr = 15 + v_ctr_dev;

controllerType = 4;
% 1.Optimal Control  2.FollowerStopper  3.PI Saturation  4.Structured Optimal Control

load('example_setup_and_controller.mat');

%Simulation
N = 20;
v_max = 30;
%Scenario
Circumference = 400;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Design the equilibrium spacing of the AV
s_star = acos(1-v_ctr./v_max*2)/pi.*(s_go-s_st)+s_st;
s_ctr = Circumference-sum(s_star(1:N-1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~mix
    ActuationTime = 99999;  %When will the controller work
else
    ActuationTime = 0;
end

acel_max = 2;
dcel_max = -5;


TotalTime = 200;
Tstep = 0.01;
NumStep = TotalTime/Tstep;

alpha_k = 0.6;



%Initial State for each vehicle
S = zeros(NumStep,N,3);

load('initial_deviation_setup.mat');
S(1,:,1) = InitialPosition;
S(1,:,2) = InitialVelocity;

ID = zeros(1,N);
if mix
    ID(N) = 1;
    X = zeros(2*N,NumStep);
end

%0. HDV  1. One AV

%Velocity Difference
V_diff = zeros(NumStep,N);
%Following Distance
D_diff = zeros(NumStep,N);
temp = zeros(N,1);
%Avg Speed
V_avg = zeros(NumStep,1);
v_cmd = zeros(NumStep,1);  % for controller 2 & 3





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
    for i = 1:N
        if cal_D(i)>s_go(i)
            cal_D(i) = s_go(i);
        elseif cal_D(i)<s_st(i)
            cal_D(i) = s_st(i);
        end
    end
    
    %OVM Model
    %V_d = v_max/2*(1-cos(pi*(h-h_st)/(h_go-h_st)));
    %a2 = alpha*(V_h-v2)+beta*(v1-v2);
    acel = alpha.*(v_max/2.*(1-cos(pi*(cal_D'-s_st)./(s_go-s_st)))-S(k,:,2)')+beta.*V_diff(k,:)';
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
                X(1:2:2*N,k) = reshape(D_diff(k,:),N,1)-s_star;
                X(2:2:2*N,k) = reshape(S(k,:,2),N,1)-v_ctr;
                X(2*N-1,k) = D_diff(k,N)-s_ctr;
                u = -K*X(:,k);
            case 4
                X(1:2:2*N,k) = reshape(D_diff(k,:),N,1)-s_star;
                X(2:2:2*N,k) = reshape(S(k,:,2),N,1)-v_ctr;
                X(2*N-1,k) = D_diff(k,N)-s_ctr;
                u = -K*X(:,k);
             case 2
                dx10 = 9.5; dx20=10.75; dx30 = 11;
                dv_temp = min(S(k,N-1,2)-S(k,N,2),0);
                d1 = 1.5; d2 = 1.0; d3 = 0.5;
                dx1 = dx10+dv_temp^2/2/d1;
                dx2 = dx20+dv_temp^2/2/d2;
                dx3 = dx30+dv_temp^2/2/d3;
                dx = D_diff(k,N);
                v_temp = min(S(k,N-1,2),12);
                if dx<=d1
                    v_cmd = 0;
                elseif dx<=d2
                    v_cmd = v_temp*(dx-dx1)/(dx2-dx1);
                elseif dx<=d3
                    v_cmd = v_temp+(15-v_temp)*(dx-dx2)/(dx3-dx2);
                else
                    v_cmd = 15;
                end
                u = alpha_k*(v_cmd-S(k,N,2));
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
                u = alpha_k*(v_cmd(k+1)-S(k,N,2));
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
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
    
    
end

for k = 1:NumStep
    V_avg(k) = mean(S(k,:,2));
end

%% Plot
Wsize = 20;
%Velocity
figure;
for i=1:N
    if ID(i) == 0
        p1 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',1,'Color',[190 190 190]/255);
    else
        p2 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[0 0.4470 0.7410]);
    end
    hold on;
end
p3 = plot(Tstep:Tstep:TotalTime,V_avg,'-','linewidth',1.5,'Color','k');


set(gca,'TickLabelInterpreter','latex','fontsize',14);
grid on;
xlabel('$t$ ($\mathrm{s}$)','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('Velocity ($\mathrm{m/s}$)','fontsize',Wsize,'Interpreter','latex','Color','k');
axis([0 100 10 20]);

if mix
    l1 = legend([p1 p2 p3],'HDV','CAV','Average velocity','Location','NorthEast');
else
    l1 = legend([p1 p3],'HDV','Average velocity','Location','NorthWest');
end
set(gcf,'Position',[850 150 450 350]);
fig = gcf;
fig.PaperPositionMode = 'auto';
set(l1,'fontsize',18,'box','off','Interpreter','latex');
% print(gcf,'Figures/Figure7a_HDV','-depsc','-r300');



