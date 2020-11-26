%% Description
% Plot the marginal improvement of ACC to check its submodularity
% Correspond to Fig. 6 in the paper

clc;
close all;
clear;

OVM_bool = 1; % 0 or 1
controller = 2; % 2: kv=3,ks=0.3; 3: kv=1,ks=0.1

%% 

if OVM_bool
    load(['..\_data\Submodularity\ACC\OVM_10-May-2020_N_12_Controller_',num2str(controller),'.mat'])
else
    load(['..\_data\Submodularity\ACC\NotOVM_10-May-2020_N_12_Controller_',num2str(controller),'.mat'])
end

%%

LWidth = 1.5;
FontSize = 20;
MSize = 8;

FiveColors = [168 33 157;
    242 60 84;
    242 177 36;
    12 249 162;
    12 173 242]/255;

if OVM_bool
switch controller
    case 2
plot(Gain(1,:),'Color',FiveColors(1,:),'linewidth',LWidth); hold on;
p1 = plot(Gain(1,:),'o','Color',FiveColors(1,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(2,:),'Color',FiveColors(2,:),'linewidth',LWidth); hold on;
p2 = plot(Gain(2,:),'^','Color',FiveColors(2,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(3,:),'Color',FiveColors(3,:),'linewidth',LWidth); hold on;
p3 = plot(Gain(3,:),'s','Color',FiveColors(3,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(6,:),'Color',FiveColors(4,:),'linewidth',LWidth); hold on;
p4 = plot(Gain(6,:),'x','Color',FiveColors(4,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(7,:),'Color',FiveColors(5,:),'linewidth',LWidth); hold on;
p5 = plot(Gain(7,:),'d','Color',FiveColors(5,:),'linewidth',1.5,'markersize',MSize);
    case 3
        plot(Gain(1,:),'Color',FiveColors(1,:),'linewidth',LWidth); hold on;
p1 = plot(Gain(1,:),'o','Color',FiveColors(1,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(2,:),'Color',FiveColors(2,:),'linewidth',LWidth); hold on;
p2 = plot(Gain(2,:),'^','Color',FiveColors(2,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(3,:),'Color',FiveColors(3,:),'linewidth',LWidth); hold on;
p3 = plot(Gain(3,:),'s','Color',FiveColors(3,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(4,:),'Color',FiveColors(4,:),'linewidth',LWidth); hold on;
p4 = plot(Gain(4,:),'x','Color',FiveColors(4,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(5,:),'Color',FiveColors(5,:),'linewidth',LWidth); hold on;
p5 = plot(Gain(5,:),'d','Color',FiveColors(5,:),'linewidth',1.5,'markersize',MSize);
end
else
    switch controller
    case 2
        plot(Gain(1,:),'Color',FiveColors(1,:),'linewidth',LWidth); hold on;
p1 = plot(Gain(1,:),'o','Color',FiveColors(1,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(2,:),'Color',FiveColors(2,:),'linewidth',LWidth); hold on;
p2 = plot(Gain(2,:),'^','Color',FiveColors(2,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(3,:),'Color',FiveColors(3,:),'linewidth',LWidth); hold on;
p3 = plot(Gain(3,:),'s','Color',FiveColors(3,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(6,:),'Color',FiveColors(4,:),'linewidth',LWidth); hold on;
p4 = plot(Gain(6,:),'x','Color',FiveColors(4,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(7,:),'Color',FiveColors(5,:),'linewidth',LWidth); hold on;
p5 = plot(Gain(7,:),'d','Color',FiveColors(5,:),'linewidth',1.5,'markersize',MSize);

    case 3
        plot(Gain(1,:),'Color',FiveColors(1,:),'linewidth',LWidth); hold on;
p1 = plot(Gain(1,:),'o','Color',FiveColors(1,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(2,:),'Color',FiveColors(2,:),'linewidth',LWidth); hold on;
p2 = plot(Gain(2,:),'^','Color',FiveColors(2,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(3,:),'Color',FiveColors(3,:),'linewidth',LWidth); hold on;
p3 = plot(Gain(3,:),'s','Color',FiveColors(3,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(6,:),'Color',FiveColors(4,:),'linewidth',LWidth); hold on;
p4 = plot(Gain(6,:),'x','Color',FiveColors(4,:),'linewidth',1.5,'markersize',MSize);
plot(Gain(5,:),'Color',FiveColors(5,:),'linewidth',LWidth); hold on;
p5 = plot(Gain(5,:),'d','Color',FiveColors(5,:),'linewidth',1.5,'markersize',MSize);
set(gca,'ylim',[3.9e-3,4.7e-3]);
    end    
end


l = legend([p1,p2,p3,p4,p5],{'Case 1','Case 2','Case 3','Case 4','Case 5'});
l.FontSize = 16;
l.Interpreter = 'latex';
l.Box = 'off';

switch controller
    case 1
        l.Location = 'NorthEast';
    case 2
        l.Location = 'NorthEast';
    case 4
        %         l.Location = 'SouthWest';
        l.Position = [0.2,0.2421,0.3282,0.3840];
    case 5
        %         l.Location = 'SouthWest';
        l.Position = [0.2,0.2421,0.3282,0.3840];
        
        
end

set(gca,'TickLabelInterpreter','latex','fontsize',16);
grid on;

set(gca,'XLim',[1,size(Gain,2)]);

xl = xlabel('$\vert S_k \vert $','fontsize',FontSize,'Interpreter','latex','Color','k');
yl = ylabel('$\triangle_{J_1} ( 1 | S_k )$','fontsize',FontSize,'Interpreter','latex','Color','k');

set(gcf,'Position',[250,150,400,330]);
fig = gcf;
fig.PaperPositionMode = 'auto';

% print(gcf,['Figs/Fig6_Submodular_OVM_',num2str(OVM_bool),'_Controller_',num2str(controller)],'-depsc','-r300');
