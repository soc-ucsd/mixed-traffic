%% Discription
% Analyze the influence of AV number and system size on the performance.
% Plot the results from random simulations.

clc;
clear;
close all;


addpath('..\_data');

%% 1AV & 2AV
Wsize = 18;
LWidth = 1.3;
MSize  = 7;
Lname1 = 'One AV';
Lname2 = 'Two AVs';
% Time to stabilize
figure;
load('InitialDeviation_SystemSize_1AV.mat');
id = Index(find(Index>0));
ct1 = ConvergenceTime(find(Index>0));
plot(id,ct1,'k','linewidth',LWidth); hold on;
p1 = plot(id,ct1,'k^','linewidth',1.1,'markersize',MSize,'displayname',Lname1);

hold on;
load('InitialDeviation_SystemSize_2AV.mat');
id = Index(find(Index>0));
ct2 = ConvergenceTime(find(Index>0));
plot(id,ct2,'m','linewidth',LWidth);
p2 = plot(id,ct2,'mo','linewidth',1.1,'markersize',MSize,'displayname',Lname2);
set(gca,'TickLabelInterpreter','latex','fontsize',16,'ylim',[0,160],'xtick',0:20:100,'ytick',0:40:160);
xlabel('Number of vehicles','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('Time to stabilize $[s]$','fontsize',Wsize,'Interpreter','latex','Color','k');


grid on;box on;
l = legend([p1 p2]);
%l.Position = [0.3 0.75 0.125 0.1];
l.Location = 'NorthWest';
l.Interpreter = 'latex';
l.FontSize = Wsize-2;
l.Box = 'off';


set(gcf,'Position',[250 150 390 310]);

fig = gcf;
fig.PaperPositionMode = 'auto';
% print(gcf,['figs/TimeToStabilize_Comparison_yz'],'-painters','-depsc2','-r300')

% Control Energy for AV
figure;
load('InitialDeviation_SystemSize_1AV.mat');
plot(Index(find(Index>0)),controlEnergy(find(Index>0)),'k','linewidth',LWidth); hold on;
p1 = plot(Index(find(Index>0)),controlEnergy(find(Index>0)),'k^','linewidth',1.1,'markersize',MSize,'displayname',Lname1);

hold on;
load('InitialDeviation_SystemSize_2AV.mat');
plot(Index(find(Index>0)),controlEnergy(find(Index>0))./2,'m','linewidth',LWidth);
p2 = plot(Index(find(Index>0)),controlEnergy(find(Index>0))./2,'mo','linewidth',1.1,'markersize',MSize,'displayname',Lname2);
set(gca,'TickLabelInterpreter','latex','fontsize',16,'ylim',[0,12],'xtick',0:20:100,'ytick',0:3:12);
xlabel('Number of vehicles','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('Control energy','fontsize',Wsize,'Interpreter','latex','Color','k');


grid on;box on;
l = legend([p1 p2]);
%l.Position = [0.3 0.75 0.125 0.1];
l.Location = 'NorthWest';
l.Interpreter = 'latex';
l.FontSize = Wsize-2;
l.Box = 'off';

set(gcf,'Position',[650 150 390 310]);
fig = gcf;
fig.PaperPositionMode = 'auto';
% print(gcf,['figs/ControlEnergy_Comparison_yz'],'-painters','-depsc2','-r300')