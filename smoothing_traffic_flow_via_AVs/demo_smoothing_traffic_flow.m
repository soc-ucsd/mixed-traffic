%% Discription
% Traffic simulation when a sudden perturbation happens at one vehicle
% Homogeneous setup for HDVs
% Initial velocity and position are in equilibrium

clc;
clear;
close all;
addpath('_fcn');
addpath('_data');

%% Key parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mix or not
mix = 1;
% 1.Optimal Control  2.FollowerStopper  3.PI with Saturation
controllerType = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

brakeID = 6 - 1;
% In the simulation, the ID of the AV is 20, and thus the brakeID needs to minus one

N = 20;
s_star = 20;

videoOutput = 0; % whether write into the video


%% Controller
if mix && controllerType == 1
    %Cost Function Weight
    gammaType = 1;
    switch gammaType
        case 1
            gamma_s = 0.03;
            gamma_v = 0.15;
            gamma_u = 1;
            
        case 2
            gamma_s = 3;
            gamma_v = 15;
            gamma_u = 1;
    end
    K = lqr_sdp(N,s_star,gamma_s,gamma_v,gamma_u,1);
end

alpha_k = 0.6; % For FollowerStopper


%% Parameters
v_max = 30;
acel_max = 5;
dcel_max = -5;
%Driver Model: OVM
alpha = 0.6;
beta = 0.9;
s_st = 5;
s_go = 35;

%Animation
velUpperBound = 15; % color
velLowerBound = 8; % color
vehicleSize = 12; % MarkerSize

%Simulation

TotalTime = 100;
Tstep = 0.01;
NumStep = TotalTime/Tstep;
%Scenario
Circumference = s_star*N;
if mix
    ActuationTime = 0;  %When will the controller work
else
    ActuationTime = 9999;
end
%Equilibrium
%s_star = 0.5*(s_st+s_go);
s_star = Circumference/N;
v_star = v_max/2*(1-cos(pi*(s_star-s_st)/(s_go-s_st))); %THEORETICALLY equilibrium velocity
s_ctr = s_star*1.0;
v_ctr = v_max/2*(1-cos(pi*(s_ctr-s_st)/(s_go-s_st)));

% safe distance for collision avoidance
sd = 0; % minimum value is zero since the vehicle length is ignored

%Initial State for each vehicle
S = zeros(NumStep,N,3);
dev_s = 0;
dev_v = 0;
co_v = 1.0;
v_ini = co_v*v_star; %Initial velocity
%from -dev to dev


S(1,:,1) = linspace(Circumference,s_star,N)'+(rand(N,1)*2*dev_s-dev_s);  %The vehicles are uniformly distributed on the ring road with a random deviation
S(1,:,2) = v_ini*ones(N,1)+(rand(N,1)*2*dev_v-dev_v);


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




%% Theoretical Analysis

disp(['Vehicle Number = ',num2str(N)]);
disp(['v_star = ',num2str(v_star)]);
disp(['s_star = ',num2str(s_star)]);
disp(['v_ini = ',num2str(v_ini)]);
disp(['controllerType = ',num2str(controllerType)]);
if (v_max*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st))-alpha-2*beta)<=0
    disp('Satisfy Sufficient Condition!');
else
    disp('Not Satisfy Sufficient Condition!');
end


%% Simulation begins
sd_actuate = 0;
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
    
    
    if (k*Tstep>20)&&(k*Tstep<22)
        S(k,brakeID,3)=-5;
    end
    
    if k>ActuationTime/Tstep
        switch controllerType
            case 1
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
                    v_cmd = v_temp+(v_ctr-v_temp)*(dx-dx2)/(dx3-dx2);
                else
                    v_cmd = v_ctr;
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
        
        if (S(k,N,2)^2-S(k,N-1,2)^2)/2/(S(k,N-1,1)-S(k,N,1)-sd)>abs(dcel_max)
            
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

figuresize = [250 150 400 300];
Lwidth = 1.2;

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
Wsize = 16;

zq = griddata(x,y,z,xq,yq);
mesh(xq,yq,zq);
view([0 90]);
colormap(hot);
%colormap(flipud(colormap));
caxis([0 15]);
hcb = colorbar;
hcb.TickLabelInterpreter = 'latex';
hcb.FontSize = 14;
set(gca,'TickLabelInterpreter','latex','fontsize',14);
xlabel('$t\;[\mathrm{s}]$','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('Position $[\mathrm{m}]$','fontsize',Wsize,'Interpreter','latex','Color','k');
clabel = get(hcb,'label');
set(clabel,'String','Velocity $[\mathrm{m/s}]$','Interpreter','latex','fontsize',Wsize,'Color','k');


% plot brakeID vehicle and AV
hold on;
for i=1:N
    if ID(i) == 0
        plot3(Tstep:Tstep:TotalTime,mod(S(:,i,1),Circumference),30*ones(TotalTime/Tstep,1),'.','color',[190 190 190]/255,'markersize',3);
        %For Label
        p1 = plot3(1:3,2*Circumference*ones(3,1),ones(3,1),'color',[190 190 190]/255,'linewidth',Lwidth);
    else
        plot3(Tstep:Tstep:TotalTime,mod(S(:,i,1),Circumference),30*ones(TotalTime/Tstep,1),'.','color',[0 0.4470 0.7410],'markersize',3);
        %For Label
        p2 = plot3(1:3,2*Circumference*ones(3,1),ones(3,1),'color',[0 0.4470 0.7410],'linewidth',Lwidth);
    end
    hold on;
end
%Brake Time
plot3(20:Tstep:22,mod(S(20/Tstep:22/Tstep,brakeID,1),Circumference),30*ones(2/Tstep+1,1),'.','color',[0.6350 0.0780 0.1840],'markersize',3);
%scatter3(Tstep:Tstep:TotalTime,mod(S(:,brakeID,1),Circumference),30*ones(TotalTime/Tstep,1),4,'k','filled');

axis([0 80 0 Circumference]);

set(gcf,'Position',figuresize);
fig = gcf;
fig.PaperPositionMode = 'auto';
% print(gcf,['figs/StrongPerturbationAV_1'],'-depsc2','-r600');

%Velocity
figure;
for i=1:N
    if ID(i) == 0
        p1 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',1,'Color',[190 190 190]/255);
    else
        p2 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',Lwidth,'Color',[0 0.4470 0.7410]);
    end
    hold on;
end
p3 = plot(Tstep:Tstep:TotalTime,V_avg,'-','linewidth',Lwidth,'Color','k');

%Brake Time
plot(20:Tstep:22,S(20/Tstep:22/Tstep,brakeID,2),'-','linewidth',Lwidth,'color',[0.6350 0.0780 0.1840]);

set(gca,'TickLabelInterpreter','latex','fontsize',14);
grid on;
xlabel('$t\;[\mathrm{s}]$','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('Velocity $[\mathrm{m/s}]$','fontsize',Wsize,'Interpreter','latex','Color','k');
axis([0 80 0 30]);

if mix
    l1 = legend([p1 p2 p3],'OVM','Autonomous vehicle','Average velocity','Location','NorthEast');
else
    l1 = legend([p1 p3],'OVM','Average velocity','Location','NorthEast');
end
set(gcf,'Position',figuresize+[500 0 0 0]);
fig = gcf;
fig.PaperPositionMode = 'auto';
set(l1,'fontsize',Wsize-2,'box','off','Interpreter','latex','location','NorthWest');
% print(gcf,['figs/StrongPerturbationAV_2'],'-painters','-depsc2','-r600')




%% Animation

if videoOutput
myVideo = VideoWriter(videoFile,'MPEG-4');
open(myVideo);
end

load('Animation_Color.mat');

figure(3);
set(0,'defaultfigurecolor','w');
ax1 = subplot(1,2,1);
% vehicle position
R = Circumference/2/pi;
for id = 1:N
    position(id) = plot(R*cos(S(1,id,1)/Circumference*2*pi),R*sin(S(1,id,1)/Circumference*2*pi),'o');
    position(id).MarkerSize = vehicleSize;
    if ~mix
        position(id).MarkerFaceColor = [146,208,80]/255;
        position(id).MarkerEdgeColor = [0,0,0];
    else
        if id == 20
            position(id).MarkerFaceColor = [0,176,240]/255;
            position(id).MarkerEdgeColor = [0,0,0];
        else
            position(id).MarkerFaceColor = [146,208,80]/255;
            position(id).MarkerEdgeColor = [0,0,0];
        end
    end
    hold on;
end
axis equal;
axis manual;
axis([-R-20,R+20,-R-20,R+20]);
set(gcf,'Position',[150,100,1000,450]);
axis off;
% Color
colormap(velocityMap);

hcb = colorbar;
clabel = get(hcb,'label');
caxis([velLowerBound,velUpperBound]);
set(clabel,'String','Velocity $[\mathrm{m/s}]$','Interpreter','latex','fontsize',12);
hcb.TickLabelInterpreter = 'latex';
hcb.FontSize = 10;
hcb.Position = [0.13 0.25 0.01 0.5];
% Road
temp = linspace(0,2*pi,100);
plot(0.87*R*cos(temp),0.87*R*sin(temp),'k','LineWidth',0.4);
plot(1.13*R*cos(temp),1.13*R*sin(temp),'k','LineWidth',0.4);

ax2 = subplot(1,2,2);
for id = 1:N
    velocity(id) = animatedline('Color',[190 190 190]/255,'linewidth',1);
    if mix
        velocity(20) = animatedline('Color',[0 0.4470 0.7410],'linewidth',1.2);
    end
end
axis manual;
axis([10 80 0 30]);
set(gca,'TickLabelInterpreter','latex','fontsize',10);
xlabel('$t\;[\mathrm{s}]$','fontsize',12,'Interpreter','latex');
ylabel('Velocity $[\mathrm{m/s}]$','fontsize',12,'Interpreter','latex');

fig = gcf;
fig.Children(1).Position = [0.52 0.26 0.3 0.5];

%% Update

for i=10/Tstep:10:(TotalTime-20)/Tstep
    
    for id = 1:N
        ax1;
        position(id).XData = R*cos(S(i,id,1)/Circumference*2*pi);
        position(id).YData = R*sin(S(i,id,1)/Circumference*2*pi);
        
        if S(i,id,2)<velLowerBound
            temp = velLowerBound;
        elseif S(i,id,2)>velUpperBound
            temp = velUpperBound;
        else
            temp = S(i,id,2);
        end
        colorID = min(floor((temp-velLowerBound)/(velUpperBound-velLowerBound)*size(velocityMap,1))+1,size(velocityMap,1));
        position(id).MarkerFaceColor = velocityMap(colorID,:);
        if mix
            position(20).MarkerFaceColor = [0,176,240]/255;
        end
        ax2;
        addpoints(velocity(id),i*Tstep,S(i,id,2));
    end
    frame = getframe(gcf);
    if videoOutput
    writeVideo(myVideo,frame);
    end
    drawnow;
    pause(0.05);
    %pause(0.1);
    
end
if videoOutput
close(myVideo);
end