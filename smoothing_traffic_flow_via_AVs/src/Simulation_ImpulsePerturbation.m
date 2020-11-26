%% Discription
% Traffic simulation when a slight impulse perturbation happens at one vehicle
% Homogeneous setup for HDVs

clc;
clear;
close all;

%% Key parameters

PerID = 1; %which vehicle has an initial perturbation



%% Parameters
v_max = 30;
acel_max = 5;
dcel_max = -5;
%Driver Model: OVM
alpha = 0.6;
beta = 0.9;
s_st = 5;
s_go = 35;

s_star = 20;
v_star = v_max/2*(1-cos(pi*(s_star-s_st)/(s_go-s_st))); %THEORETICALLY equilibrium velocity
s_ctr = s_star*1.0;
v_ctr = v_max/2*(1-cos(pi*(s_ctr-s_st)/(s_go-s_st)));

%Simulation
N = 20;
TotalTime = 100;
Tstep = 0.1;
NumStep = TotalTime/Tstep;
%Scenario
Circumference = s_star*N;

%Initial State for each vehicle
S{1} = zeros(NumStep,N,3);
S{2} = zeros(NumStep,N,3);
S{3} = zeros(NumStep,N,3);
% S{Case}(NumStep,vehicleNumber,1:Position 2:Velocity 3:Accleration)


% 1. all HDVs  2. one AV with CACC  3.one AV with optimal control
for controlCase = 1:3
    
    co_v = 1.0;
    v_ini = co_v*v_star; %Initial velocity
    
    S{controlCase}(1,:,1) = linspace(Circumference,s_star,N)';
    S{controlCase}(1,:,2) = v_ini*ones(N,1);
    
    S{controlCase}(1,PerID,2) = S{controlCase}(1,PerID,2);
    
    
    ID = zeros(1,N);
    %0. Manually Driven  1. Controller
    % Mix or not
    mix = 0;
    if controlCase > 1
        mix = 1;
        ID(N) = 1;
        X = zeros(2*N,NumStep);
    end
    
    
    %Velocity Difference
    V_diff = zeros(NumStep,N);
    %Following Distance
    D_diff = zeros(NumStep,N);
    temp = zeros(N,1);
    %Avg Speed
    V_avg = zeros(NumStep,1);
    
    
    %% Theoretical Analysis
    
    disp(['Vehicle Number = ',num2str(N)]);
    disp(['v_star = ',num2str(v_star)]);
    disp(['s_star = ',num2str(s_star)]);
    disp(['v_ini = ',num2str(v_ini)]);
    disp(['Type = ',num2str(controlCase)]);
    if (v_max*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st))-alpha-2*beta)<=0
        disp('Satisfy Sufficient Condition!');
    else
        disp('Not Satisfy Sufficient Condition!');
    end
    %% Controller
    if controlCase == 3
        gamma_s = 0.03;
        gamma_v = 0.15;
        gamma_u = 1;
        K = lqr_sdp(N,s_star,gamma_s,gamma_v,gamma_u,1);
    end
    %% Simulation begins
    
    
    
    for k = 1:NumStep
        %Update acceleration
        temp(2:end) = S{controlCase}(k,1:(end-1),2);
        temp(1) = S{controlCase}(k,end,2);
        V_diff(k,:) = temp-reshape(S{controlCase}(k,:,2),N,1);
        temp(1) = S{controlCase}(k,end,1)+Circumference;
        temp(2:end) = S{controlCase}(k,1:(end-1),1);
        D_diff(k,:) = temp-reshape(S{controlCase}(k,:,1),N,1); %Real Following Distance
        cal_D = D_diff(k,:); %For the boundary of Optimal Veloicity Calculation
        cal_D(cal_D>s_go) = s_go;
        cal_D(cal_D<s_st) = s_st;
        %OVM Model
        %V_d = v_max/2*(1-cos(pi*(h-h_st)/(h_go-h_st)));
        %a2 = alpha*(V_h-v2)+beta*(v1-v2);
        acel = alpha*(v_max/2*(1-cos(pi*(cal_D-s_st)/(s_go-s_st)))-S{controlCase}(k,:,2))+beta*V_diff(k,:);
        acel(acel>acel_max)=acel_max;
        acel(acel<dcel_max)=dcel_max;
        % SD as ADAS to prevent crash
        temp(2:end) = S{controlCase}(k,1:(end-1),2);
        temp(1) = S{controlCase}(k,end,2); %temp is the velocity of the preceding vehicle
        acel_sd = (reshape(S{controlCase}(k,:,2).^2,N,1)-temp.^2)./2./reshape(D_diff(k,:),N,1);
        acel(acel_sd>abs(dcel_max)) = dcel_max;
        
        S{controlCase}(k,:,3) = acel;
        
        %Initial Perturbation to the vehicle following AV
        if k*Tstep<1
            S{controlCase}(k,PerID,3) = 1;
        elseif k*Tstep<2
            S{controlCase}(k,PerID,3) = -1;
        end
        
        if mix
            switch controlCase
                case 2
                    %s_star/v_star = 4/3 = time headway for ACC;
                    u = 1*(S{controlCase}(k,N-1,1)-S{controlCase}(k,N,1)-S{controlCase}(k,N,2)*4/3)+2*(S{controlCase}(k,N-1,2)-S{controlCase}(k,N,2))+1*(S{controlCase}(k,N-1,3)-S{controlCase}(k,N,3));
                    
                case 3
                    X(1:2:2*N,k) = reshape(D_diff(k,:),N,1)-s_ctr;
                    X(2:2:2*N,k) = reshape(S{controlCase}(k,:,2),N,1)-v_ctr;
                    u = -K*X(:,k);
                    
            end
            %         if u>acel_max
            %             u=acel_max;
            %         elseif u<dcel_max
            %             u=dcel_max;
            %         end
            
            if (S{controlCase}(k,N,2)^2-S{controlCase}(k,N-1,2)^2)/2/(S{controlCase}(k,N-1,1)-S{controlCase}(k,N,1))>abs(dcel_max)
                
                u=dcel_max;
            end
            
            S{controlCase}(k,N,3) = u;
        end
        if k<NumStep
            S{controlCase}(k+1,:,2) = S{controlCase}(k,:,2) + Tstep*S{controlCase}(k,:,3);
            S{controlCase}(k+1,:,1) = S{controlCase}(k,:,1) + Tstep*S{controlCase}(k,:,2);
        end
        
    end
    
    for k = 1:NumStep
        V_avg(k) = mean(S{controlCase}(k,:,2));
    end
    
end


%% Plot Figure

Wsize = 16;  % word size

% figure 1: all HDVs
F = figure;
for i=1:N-1
    p1 = plot3((1:NumStep).*Tstep,(i+1)*ones(1,NumStep),S{1}(:,i,2) - v_star,'-','linewidth',1,'Color',[0 0.447 0.741]);
    hold on;
end
p2 = plot3((1:NumStep).*Tstep,1*ones(1,NumStep),S{1}(:,N,2) - v_star,'-','linewidth',1,'Color',[0 0.447 0.741]);
if mix
    p2.Color = [216 82 24]/255;
end
set(gca,'TickLabelInterpreter','latex','fontsize',15);
set(gca,'zlim',[-5 5]);
grid on;
x1 = xlabel('$t\;[\mathrm{s}]$','fontsize',Wsize,'Interpreter','latex','Color','k');
y1 = ylabel('Vehicle index','fontsize',Wsize,'Interpreter','latex','Color','k');
x1.Rotation = 20;
x1.Position=[48,-1.5901,-6.7148];
y1.Rotation = -30;
y1.Position=[-7.8839,6,-6.9147];
zlabel('Velocity deviation $[\mathrm{m/s}]$','fontsize',Wsize,'Interpreter','latex','Color','k');
set(gcf,'Position',[250 150 420 360]);
% print(gcf,['figs/TrafficWave_1'],'-painters','-depsc2','-r300')



% figure 2: one AV with CACC
figure;
for i=1:N-1
    p1 = plot3((1:NumStep).*Tstep,(i+1)*ones(1,NumStep),S{2}(:,i,2)-v_star,'-','linewidth',1,'Color',[0 0.447 0.741]);
    hold on;
end
p2 = plot3((1:NumStep).*Tstep,1*ones(1,NumStep),S{2}(:,N,2)-v_star,'-','linewidth',1,'Color',[0 0.447 0.741]);
if mix
    p2.Color = [216 82 24]/255;
end
set(gca,'TickLabelInterpreter','latex','fontsize',15);
set(gca,'zlim',[-5 5]);
grid on;
x1 = xlabel('$t\;[\mathrm{s}]$','fontsize',Wsize,'Interpreter','latex','Color','k');
y1 = ylabel('Vehicle index','fontsize',Wsize,'Interpreter','latex','Color','k');
x1.Rotation = 20;
x1.Position=[48,-1.5901,-6.7148];
y1.Rotation = -30;
y1.Position=[-7.8839,6,-6.9147];
zlabel('Velocity deviation $[\mathrm{m/s}]$','fontsize',Wsize,'Interpreter','latex','Color','k');
set(gcf,'Position',[650 150 420 360]);
% print(gcf,['figs/TrafficWave_2'],'-painters','-depsc2','-r300')



% figure 3: one AV with optimal control
F = figure;
for i=1:N-1
    p1 = plot3((1:NumStep).*Tstep,(i+1)*ones(1,NumStep),S{3}(:,i,2)-v_star,'-','linewidth',1,'Color',[0 0.447 0.741]);
    hold on;
end
p2 = plot3((1:NumStep).*Tstep,1*ones(1,NumStep),S{3}(:,N,2)-v_star,'-','linewidth',1,'Color',[0 0.447 0.741]);
if mix
    p2.Color = [216 82 24]/255;
end
set(gca,'TickLabelInterpreter','latex','fontsize',15);
set(gca,'zlim',[-5 5]);
grid on;
x1 = xlabel('$t\;[\mathrm{s}]$','fontsize',Wsize,'Interpreter','latex','Color','k');
y1 = ylabel('Vehicle index','fontsize',Wsize,'Interpreter','latex','Color','k');
x1.Rotation = 20;
x1.Position=[48,-1.5901,-6.7148];
y1.Rotation = -30;
y1.Position=[-7.8839,6,-6.9147];
zlabel('Velocity deviation $[\mathrm{m/s}]$','fontsize',Wsize,'Interpreter','latex','Color','k');
set(gcf,'Position',[1050 150 420 360]);
% print(gcf,['figs/TrafficWave_3'],'-painters','-depsc2','-r300')

