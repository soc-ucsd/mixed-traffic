%% Discription
% Driver Model - OVM
% Homogeneous Setting for alpha, beta and fv
% Initial velocity and position are in equilibrium
% One vehicle will Brake sharply
% Add a safe distance algorithm to guarantee safety (not crash the
% preceding vehicle), including HDV and AV
% Correspond to Fig. 12 in our paper.

clc;
clear;
close all;

addpath('_fcn');
addpath('_data');

%% Key Parameters

N = 20;
AV_number = 0; % 0 or 1 or 2 or 4

platoon_bool= 0;

% Position of the perturbation
brakeID = 15;



%% Parameters

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

% Controller Parameter
gammaType = 2;

v_star = 15;

% OVM parameter
s_star = 20;
v_max  = 30;
s_st   = 5;
s_go   = 35;
%%%%% Type1 %%%%%%%
alpha  = 0.6;
beta   = 0.9;
%%%%%%%%% Type2 %%%%%%%%%
%     alpha  = 1.0;
%     beta   = 1.5;


%% Other Parameters

acel_max = 2;
dcel_max = -5;
%Driver Model: OVM

% safe distance for collision avoidance
sd = 8; % minimum value is zero since the vehicle length is ignored

%Simulation
TotalTime = 100;
Tstep = 0.01;
NumStep = TotalTime/Tstep;
%Scenario
Circumference = s_star*N;

%Initial State for each vehicle
S = zeros(NumStep,N,3);
dev_s = 0;
dev_v = 0;
co_v = 1.0;
v_ini = co_v*v_star; %Initial velocity
%from -dev to dev


S(1,:,1) = linspace(Circumference,s_star,N)'+(rand(N,1)*2*dev_s-dev_s);
%The vehicles are uniformly distributed on the ring road with a random deviation
S(1,:,2) = v_ini*ones(N,1)+(rand(N,1)*2*dev_v-dev_v);





%Velocity Difference
V_diff = zeros(NumStep,N);
%Following Distance
D_diff = zeros(NumStep,N);
temp = zeros(N,1);
%Avg Speed
V_avg = zeros(NumStep,1);




X = zeros(2*N,NumStep);

%% Controller


alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;
if mix
[ Obj,stable_bool,stability_condition_bool,K ] = ReturnObjectiveValue(ID,N,alpha1,alpha2,alpha3,gammaType);
end

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
        if cal_D(i)>s_go
            cal_D(i) = s_go;
        elseif cal_D(i)<s_st
            cal_D(i) = s_st;
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
    
    if mix
        AV_position = find(ID==1);
        if k>ActuationTime/Tstep
            
            X(1:2:2*N,k) = reshape(D_diff(k,:),N,1)-s_star;
            X(2:2:2*N,k) = reshape(S(k,:,2),N,1)-v_star;
            u = -K*X(:,k);
            
            if ~isempty(find(u>acel_max,1))
                u(u>acel_max)=acel_max;
            elseif ~isempty(find(u<dcel_max,1))
                u(u<dcel_max)=dcel_max;
            end
            
            for i_AV = 1:AV_number
                id_AV = AV_position(i_AV);
                if (S(k,id_AV,2)^2-S(k,id_AV-1,2)^2)/2/(S(k,id_AV-1,1)-S(k,id_AV,1)-sd)>abs(dcel_max)
                    u(i_AV)=dcel_max;
                end
                S(k,id_AV,3) = u(i_AV);
            end
            
            
        end
        
        
    end
    
    if (k*Tstep>20)&&(k*Tstep<22)
            S(k,brakeID,3)=-5;
        end
    
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
    
    
end

for k = 1:NumStep
    V_avg(k) = mean(S(k,:,2));
end
%% Plot

Lwidth = 1.2;
Wsize = 20;


%% Velocity

figure;
for i=1:N
    if ID(i) == 0
        p1 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',1,'Color',[190 190 190]/255);
        hold on;
    end
    
end
for i=1:N
    if ID(i)==1
        p2 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',Lwidth,'Color',[0 0.4470 0.7410]);
    end
    hold on;
end
%     p3 = plot(Tstep:Tstep:TotalTime,V_avg,'-','linewidth',Lwidth,'Color','k');

%Brake Time
plot(20:Tstep:22,S(20/Tstep:22/Tstep,brakeID,2),'-','linewidth',Lwidth,'color',[0.6350 0.0780 0.1840]);

set(gca,'TickLabelInterpreter','latex','fontsize',14);

grid on;
xlabel('$t\;(\mathrm{s})$','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('Velocity $(\mathrm{m/s})$','fontsize',Wsize,'Interpreter','latex','Color','k');
axis([5 70 5 25]);

if mix
    % l1 = legend([p1 p2 p3],'HDV','AV','Avg. velocity','Location','NorthWest');
    l1 = legend([p1 p2],'HDV','AV','Location','NorthWest');
else
    l1 = legend([p1],'OVM','Location','NorthWest');
end

fig = gcf;
fig.PaperPositionMode = 'auto';
set(l1,'fontsize',18,'box','off','Interpreter','latex','Location','NorthEast');



set(gcf,'Position',[250 150 400 310]);

fig = gcf;
fig.PaperPositionMode = 'auto';


%% Plot Trajectory

S(:,:,1) = S(:,:,1) + 320;

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
Wsize = 20;

zq = griddata(x,y,z,xq,yq);
mesh(xq,yq,zq);
view([0 90]);

load('ColorMap_VelocityTrajectory.mat');


colormap(mymap);
caxis([5 15]);


hcb = colorbar;
xlabel('$t$ ($\mathrm{s}$)','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('Position ($\mathrm{m}$)','fontsize',Wsize,'Interpreter','latex','Color','k');
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
        plot3(Tstep:Tstep:TotalTime,mod(S(:,i,1),Circumference),30*ones(TotalTime/Tstep,1),'.','color',[0 0.4470 0.7410],'markersize',3);
        %For Label
        p2 = plot3(1:3,2*Circumference*ones(3,1),ones(3,1),'color',[0 0.4470 0.7410],'linewidth',2);
    end
    hold on;
end
%Brake Time
plot3(20:Tstep:22,mod(S(20/Tstep:22/Tstep,brakeID,1),Circumference),30*ones(2/Tstep+1,1),'.','color',[0.6350 0.0780 0.1840],'markersize',3);

axis([10 50 0 Circumference]);
set(gca,'TickLabelInterpreter','latex','fontsize',14);

xlabel('$t$ ($\mathrm{s}$)','fontsize',Wsize,'Interpreter','latex');
ylabel('Position ($\mathrm{m}$)','fontsize',Wsize,'Interpreter','latex');
set(gcf,'Position',[250 150 400 310]);

fig = gcf;
fig.PaperPositionMode = 'auto';

%% Plot Animation

videoOutput = 0; % whether write into the video
velUpperBound = 15; % color
velLowerBound = 8; % color
vehicleSize = 12; % MarkerSize

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
        if ID(id) == 1
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
    if ID(id) == 1
        velocity(id) = animatedline('Color',[0 0.4470 0.7410],'linewidth',1.2);
    else
    velocity(id) = animatedline('Color',[190 190 190]/255,'linewidth',1);  
    end
end
axis manual;
axis([10 TotalTime-20 0 30]);
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
        if ID(id) == 1
            position(id).MarkerFaceColor = [0,176,240]/255;
        else
            position(id).MarkerFaceColor = velocityMap(colorID,:);
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
