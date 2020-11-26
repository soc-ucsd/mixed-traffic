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

addpath('..\_fcn');

%% Key Parameters

N = 20;
AV_number = 4;

platoon_bool= 1;

% Position of the perturbation
brakeID = 15;

% Whether to save data
savedata = 0;
%% Parameters


mix = 1;


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

[ Obj,stable_bool,stability_condition_bool,K ] = ReturnObjectiveValue(ID,N,alpha1,alpha2,alpha3,gammaType);


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
                if (S(k,id_AV,2)^2-S(k,id_AV-1,2)^2)/2/(S(k,id_AV-1,1)-S(k,id_AV,1))>abs(dcel_max)
                    u(i_AV)=dcel_max;
                end
                S(k,id_AV,3) = u(i_AV);
            end
            
            
        end
        
        if (k*Tstep>20)&&(k*Tstep<22)
            S(k,brakeID,3)=-5;
        end
    end
    
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
    
    
end

for k = 1:NumStep
    V_avg(k) = mean(S(k,:,2));
end

if savedata
    switch platoon_bool
        %%%%%%%%%%%%%%%%%%
        % Change the data source
        case 1
            save(['..\_data\Traffic Simulation\Platoon_BrakeID_',num2str(brakeID),'_gammaType_2_N_',num2str(20),'_AV_',num2str(AV_number),'.mat']);
        case 0
            save(['..\_data\Traffic Simulation\Uniform_BrakeID_',num2str(brakeID),'_gammaType_2_N_',num2str(20),'_AV_',num2str(AV_number),'.mat']);
            %%%%%%%%%%%%%%%%%%%
    end
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
    l1 = legend([p1 p3],'OVM','Average velocity','Location','NorthWest');
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

