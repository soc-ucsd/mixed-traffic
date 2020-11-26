%% Description
% Plot the marginal improvement of Optimal Control Strategy to check its submodularity
% Correspond to Fig. 7 in the paper

clc;
close all;
clear;


gammaType = 9999; % A sufficiently small punishment on the control input
OVM_bool = 0; % 0 or 1

%% 

switch OVM_bool
    case 1
        load(['..\_data\Submodularity\Optimal Control Strategy\OVM_27-Apr-2020_N_12_GammaType_',num2str(gammaType),'.mat']);
    case 0
        load(['..\_data\Submodularity\Optimal Control Strategy\NotOVM_26-Apr-2020_N_12_GammaType_',num2str(gammaType),'.mat']);
        
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

if gammaType == 9999 && OVM_bool == 1
    
    plot(Gain(10,:),'Color',FiveColors(1,:),'linewidth',LWidth); hold on;
    p1 = plot(Gain(10,:),'o','Color',FiveColors(1,:),'linewidth',1.5,'markersize',MSize);
    plot(Gain(2,:),'Color',FiveColors(2,:),'linewidth',LWidth); hold on;
    p2 = plot(Gain(2,:),'^','Color',FiveColors(2,:),'linewidth',1.5,'markersize',MSize);
    plot(Gain(9,:),'Color',FiveColors(3,:),'linewidth',LWidth); hold on;
    p3 = plot(Gain(9,:),'s','Color',FiveColors(3,:),'linewidth',1.5,'markersize',MSize);
    plot(Gain(6,:),'Color',FiveColors(4,:),'linewidth',LWidth); hold on;
    p4 = plot(Gain(6,:),'x','Color',FiveColors(4,:),'linewidth',1.5,'markersize',MSize);
    plot(Gain(1,:),'Color',FiveColors(5,:),'linewidth',LWidth); hold on;
    p5 = plot(Gain(1,:),'d','Color',FiveColors(5,:),'linewidth',1.5,'markersize',MSize);
    
else
    
    plot(Gain(10,:),'Color',FiveColors(1,:),'linewidth',LWidth); hold on;
    p1 = plot(Gain(10,:),'o','Color',FiveColors(1,:),'linewidth',1.5,'markersize',MSize);
    plot(Gain(2,:),'Color',FiveColors(2,:),'linewidth',LWidth); hold on;
    p2 = plot(Gain(2,:),'^','Color',FiveColors(2,:),'linewidth',1.5,'markersize',MSize);
    plot(Gain(9,:),'Color',FiveColors(3,:),'linewidth',LWidth); hold on;
    p3 = plot(Gain(9,:),'s','Color',FiveColors(3,:),'linewidth',1.5,'markersize',MSize);
    plot(Gain(6,:),'Color',FiveColors(4,:),'linewidth',LWidth); hold on;
    p4 = plot(Gain(6,:),'x','Color',FiveColors(4,:),'linewidth',1.5,'markersize',MSize);
    plot(Gain(8,:),'Color',FiveColors(5,:),'linewidth',LWidth); hold on;
    p5 = plot(Gain(8,:),'d','Color',FiveColors(5,:),'linewidth',1.5,'markersize',MSize);
    
end

l = legend([p1,p2,p3,p4,p5],{'Case 1','Case 2','Case 3','Case 4','Case 5'});
l.FontSize = 16;
l.Interpreter = 'latex';
l.Box = 'off';
% 
% switch controller
%     case 1
%         l.Location = 'NorthEast';
%     case 2
%         l.Location = 'NorthEast';
%   
% end

set(gca,'TickLabelInterpreter','latex','fontsize',16);
grid on;





set(gca,'XLim',[1,size(Gain,2)]);

xl = xlabel('$\vert S_k \vert $','fontsize',FontSize,'Interpreter','latex','Color','k');
yl = ylabel('$\triangle_{J} (1 | S_k)$','fontsize',FontSize,'Interpreter','latex','Color','k');

set(gcf,'Position',[250,150,400,330]);
fig = gcf;
fig.PaperPositionMode = 'auto';

% print(gcf,['Figs/Fig7_Submodular_OVM_',num2str(OVM_bool)],'-depsc','-r300');

