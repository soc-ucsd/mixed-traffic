%% Discription
% Heterogeneous setup for HDVs
% At the beginning, the traffic flow is in equilibrium
% One vehicle is under a sudden perturbation
% Correspond to Fig. 9 and Fig. 10 in the following paper
% Controllability Analysis and Optimal Control of Mixed Traffic Flow With Human-Driven and Autonomous Vehicles

clc;
clear;
close all;


load('..\_data\example_setup_and_controller.mat');

%% Parameters
% Mix or not
mix = 1;

controllerType = 2;
% 1.Optimal Control  2.FollowerStopper  3.PI Saturation  4.Structured Optimal Control

brakeID = 6 - 1;
% In the simulation, the ID of the AV is 20, and thus the brakeID needs to minus one
%% Parameters

if ~mix
    ActuationTime = 99999;  %When will the controller work
else
    ActuationTime = 0;
end

acel_max = 2;
dcel_max = -5;
%Driver Model: OVM
alpha_k = 0.6;



%Simulation
N = 20;
v_max = 30;
TotalTime = 200;
Tstep = 0.01;
NumStep = TotalTime/Tstep;
%Scenario
Circumference = 400;


s_ctr = Circumference-sum(s_star(1:N-1));
v_ctr = 15;



%Initial State for each vehicle
S = zeros(NumStep,N,3);
dev_s = 0;
dev_v = 0;
co_v = 1.0;
v_ini = co_v*v_star; %Initial velocity
%from -dev to dev

S(1,1,1) = 400;
for i = 2:N-1
    S(1,i,1) = 400-sum(s_star(2:i));
end
S(1,N,1) = s_star(1);
S(1,:,2) = v_ini.*ones(N,1)+(rand(N,1)*2*dev_v-dev_v);






ID = zeros(1,N);
if mix
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
    if (k*Tstep>20)&&(k*Tstep<23)
        S(k,brakeID,3)=-3;
    end
    
    if k>ActuationTime/Tstep
        switch controllerType
            case 1
                X(1:2:2*N,k) = reshape(D_diff(k,:),N,1)-s_ctr;
                X(2:2:2*N,k) = reshape(S(k,:,2),N,1)-v_ctr;
                u = -K*X(:,k);
            case 4
                X(1:2:2*N,k) = reshape(D_diff(k,:),N,1)-s_star;
                X(2:2:2*N,k) = reshape(S(k,:,2),N,1)-v_star;
                X(2*N-1,k) = D_diff(k,N)-s_ctr;
                u = -K*X(:,k);
            case 5
                X(1:2:2*N,k) = reshape(D_diff(k,:),N,1)-s_ctr;
                X(2:2:2*N,k) = reshape(S(k,:,2),N,1)-v_ctr;
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

%% Plot Trajectory
Wsize = 20;
%Position

figure;
[xq, yq] = meshgrid(0:0.1:TotalTime,0:1:Circumference);
x = repmat(Tstep:Tstep:TotalTime,1,N);
y = mod(S(:,1,1),Circumference)';
for i=2:N
    y = [y mod(S(:,i,1),Circumference)'];
end
z = S(:,1,2)';
for i=2:N
    z = [z S(:,i,2)'];
end


zq = griddata(x,y,z,xq,yq);
mesh(xq,yq,zq);
view([0 90]);
mymap = [1,1,1;
    1,0.761,0.729;
    1,0.612,0.561;
    1,0.486,0.42;
    1,0.38,0.298;
    0.914,0.235,0.145];
colormap(mymap);
colormap(flipud(colormap));
%colormap(jet);
caxis([2 15]);
hcb = colorbar;
xlabel('$t$ ($s$)','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('Position ($m$)','fontsize',Wsize,'Interpreter','latex','Color','k');
clabel = get(hcb,'label');
set(clabel,'String','Velocity ($\mathrm{m/s}$)','fontsize',Wsize,'Interpreter','latex','Color','k');
hcb.TickLabelInterpreter = 'latex';


% plot brakeID vehicle and AV
hold on;
for i=1:N
    if ID(i) == 0
        plot3(Tstep:Tstep:TotalTime,mod(S(:,i,1),Circumference),30*ones(TotalTime/Tstep,1),'.','color',[190 190 190]/255,'markersize',2);
        %For Label
        p1 = plot3(1:3,2*Circumference*ones(3,1),ones(3,1),'color',[190 190 190]/255,'linewidth',2);
    else
        plot3(Tstep:Tstep:TotalTime,mod(S(:,i,1),Circumference),30*ones(TotalTime/Tstep,1),'.','color',[0 0.4470 0.7410],'markersize',2);
        %For Label
        p2 = plot3(1:3,2*Circumference*ones(3,1),ones(3,1),'color',[0 0.4470 0.7410],'linewidth',2);
    end
    hold on;
end
%Brake Time
plot3(20:Tstep:22,mod(S(20/Tstep:22/Tstep,brakeID,1),Circumference),30*ones(2/Tstep+1,1),'.','color',[0.6350 0.0780 0.1840],'markersize',3);

axis([0 80 0 Circumference]);
set(gca,'TickLabelInterpreter','latex','fontsize',14);
xlabel('$t$ ($\mathrm{s}$)','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('Position ($\mathrm{m}$)','fontsize',Wsize,'Interpreter','latex','Color','k');
set(gcf,'Position',[250 150 450 350]);
fig = gcf;
fig.PaperPositionMode = 'auto';

%% Plot Spacing Error
Wsize = 20;

Spacing = zeros(N,NumStep);
for i = 2:N
   Spacing(i,:) = S(:,i-1,1)-S(:,i,1); 
end
Spacing(1,:) = S(:,N,1)+Circumference-S(:,1,1);
% the final spacing of AV is regarded as the desired spacing of the AV
id_av = find(ID==1);


% Spacing Error
figure;
for i=1:N
    if controllerType == 6
         p1 = plot3(Tstep:Tstep:100,(i+1)*ones(1,100/Tstep),Spacing(i,1:100/Tstep)-(Spacing(i,1)-20),'-','linewidth',1.5,'Color',[190 190 190]/255);
         hold on;
    else

    if ID(i) == 0
        p1 = plot3(Tstep:Tstep:100,(i+1)*ones(1,100/Tstep),Spacing(i,1:100/Tstep)-(Spacing(i,end)-20),'-','linewidth',1.5,'Color',[190 190 190]/255);
    else
        p2 = plot3(Tstep:Tstep:100,1*ones(1,100/Tstep),Spacing(i,1:100/Tstep)-(Spacing(i,10000)-20),'-','linewidth',1.5,'Color',[0 0.4470 0.7410]);
    end
    hold on;
    end
end
set(gca,'TickLabelInterpreter','latex','fontsize',14);
set(gca,'zlim',[0 60]);
set(gca,'xlim',[0 100]);
set(gca,'ylim',[0 20]);
ax = gca;               % get the current axis
ax.Clipping = 'off';    % turn clipping off

x1 = xlabel('$t\;(\mathrm{s})$','fontsize',Wsize,'Interpreter','latex','Color','k');
y1 = ylabel('Vehicle index','fontsize',Wsize,'Interpreter','latex','Color','k');
z1 = zlabel('Spacing$\;(\mathrm{m})$','fontsize',Wsize,'Interpreter','latex','Color','k');
y1.Rotation = -26;
y1.Position(2)=5;
x1.Rotation = 14;
x1.Position(1)=50;
set(gcf,'Position',[850 150 450 350]);


