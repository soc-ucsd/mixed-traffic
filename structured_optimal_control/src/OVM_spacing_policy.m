%% Description
% Plot three cases of the spacing policy of OVM.
% Correspond to Fig. 2(b) in the following paper
% Controllability Analysis and Optimal Control of Mixed Traffic Flow With Human-Driven and Autonomous Vehicles

clc;
close all;
clear;

%% 

alpha = 0.6;
beta = 0.9;

s_st = 5;
s_go1 = 25;
s_go2 = 32.5;
s_go3 = 40;

v_max = 30;

s = 0:0.1:45;
v1 = zeros(size(s));
v2 = zeros(size(s));
v3 = zeros(size(s));

v1 = v_max/2*(1-cos(pi*(s-s_st)/(s_go1-s_st)));
v1(find(s<s_st)) = 0;
v1(find(s>s_go1)) = v_max;
v2 = v_max/2*(1-cos(pi*(s-s_st)/(s_go2-s_st)));
v2(find(s<s_st)) = 0;
v2(find(s>s_go2)) = v_max;
v3 = v_max/2*(1-cos(pi*(s-s_st)/(s_go3-s_st)));
v3(find(s<s_st)) = 0;
v3(find(s>s_go3)) = v_max;

i1 = find(v1>20,1);
i2 = find(v2>20,1);
i3 = find(v3>20,1);

f1 = figure(1);
%plot(s,v1,'--','linewidth',2,'color',[43,76,111]/255);
p1 = plot(s,v1,'--','linewidth',2,'color',[74,136,200]/255);
hold on;
%plot(s,v2,'-.','linewidth',2,'color',[58,114,172]/255);
p2 = plot(s,v2,'-.','linewidth',2,'color',[74,136,200]/255);
%plot(s,v3,'-','linewidth',2,'color',[74,136,200]/255);
p3 = plot(s,v3,'-','linewidth',2,'color',[74,136,200]/255);
plot(s,v_max*ones(size(s)),'-.k','linewidth',1);
plot(s(1:i3),20*ones(size(s(1:i3))),'-.k','linewidth',1);
plot(s(i1)*ones(1,10),linspace(0,20,10),'-.k','linewidth',1);
plot(s(i2)*ones(1,10),linspace(0,20,10),'-.k','linewidth',1);
plot(s(i3)*ones(1,10),linspace(0,20,10),'-.k','linewidth',1);
%% 

Wsize = 18;  % word size
set(gca,'TickLabelInterpreter','latex','fontsize',11);
grid off;
x1 = xlabel('Equilibrium spacing ($\mathrm{m}$)','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('Equilibrium velocity ($\mathrm{m/s}$)','fontsize',Wsize,'Interpreter','latex','Color','k');
axis([0 45 0 35]);
set(gcf,'Position',[250 150 450 300]);
set(gca,'ytick',[])
set(gca,'xtick',[])
l1 = legend([p1 p2 p3],'Driver 1','Driver 2','Driver 3','Location','SouthEast');
l1.FontSize = Wsize;
l1.Interpreter = 'latex';
l1.Box = 'Off';
%% 
text(0.5,20+1.7,'$v^*$','fontsize',Wsize,'Interpreter','latex');
text(0.5,30+1.7,'$v_\mathrm{max}$','fontsize',Wsize,'Interpreter','latex');
text(s(i1)+0.2,0+1.8,'$s_1^*$','fontsize',Wsize,'Interpreter','latex');
text(s(i2)+0.2,0+1.8,'$s_2^*$','fontsize',Wsize,'Interpreter','latex');
text(s(i3)+0.2,0+1.8,'$s_3^*$','fontsize',Wsize,'Interpreter','latex');
%% 


fig = gcf;
fig.PaperPositionMode = 'auto';
% print(gcf,['Figures/Figure2_OVMSpacingPolicy'],'-painters','-depsc2','-r300');