%% Description
% Plot the spatial formation of AVs in ring road

clc;
close all;
clear;

%% 

N = 12;

AV_color = [253,165,2]/255;
HDV_color = [0,176,240]/255;


% Spatial position of AVs in ring road
% 1. AV;  2. HDV
AV_ID = zeros(N,1);

AV_ID(1) = 1;
AV_ID(5) = 1;
AV_ID(9) = 1;
    

figure;

Circumference = 200;
R = Circumference/2/pi;
vehicleSize = 17;
textSize = 12;

%% Plot arrows

for i=1:N
    
    angle_delta = 0.04;
    r_delta = 0.03;
    
    x = [R*cos(-2*pi/N*(i-1/2)+pi/2-angle_delta),(1+r_delta)*R*cos(-2*pi/N*(i-1/2)+pi/2+angle_delta),(1-r_delta)*R*cos(-2*pi/N*(i-1/2)+pi/2+angle_delta)];
    y = [R*sin(-2*pi/N*(i-1/2)+pi/2-angle_delta),(1+r_delta)*R*sin(-2*pi/N*(i-1/2)+pi/2+angle_delta),(1-r_delta)*R*sin(-2*pi/N*(i-1/2)+pi/2+angle_delta)];
    
    pa = patch(x,y,[112,48,160]/255);
    pa.EdgeColor = [112,48,160]/255;
    hold on;
end

plot(R*cos(linspace(0,2*pi,100)),R*sin(linspace(0,2*pi,100)),'Color',[112,48,160]/255,'LineWidth',1);
hold on;
%% Plot formation


for i = 1:N
    vehicle(i) = plot(R*cos(-2*pi/N*(i-1)-pi/2),R*sin(-2*pi/N*(i-1)-pi/2),'o');
    vehicle(i).MarkerSize = vehicleSize;
    
    t = text(R*cos(-2*pi/N*(i-1)-pi/2),R*sin(-2*pi/N*(i-1)-pi/2),num2str(i),'HorizontalAlignment','center','Interpreter','latex','FontSize',textSize,'Color',[0.999 0.999 0.999]);
    if AV_ID(i) == 1
        vehicle(i).MarkerFaceColor = AV_color;
        vehicle(i).MarkerEdgeColor = AV_color;
    else
        vehicle(i).MarkerFaceColor = HDV_color;
        vehicle(i).MarkerEdgeColor = HDV_color;
    end
    hold on;
end



axis equal;
axis manual;
axis([-1.2*R,1.2*R,-1.2*R,1.2*R]);
set(gcf,'Position',[150,100,230,220]);
axis off;

fig = gcf;
fig.PaperPositionMode = 'auto';