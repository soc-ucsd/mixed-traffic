%% Description
% Plot the desired velocity V(s) and its derivative \dot{V}(s) in the typical OVM model


clc;
close all;
clear;

%% Parameters

alpha = 0.6;
beta = 0.9;
s_st = 5;
s_go = 35;
v_max = 30;


s = 0:0.1:40;
v = v_max/2*(1-cos(pi*(s-s_st)/(s_go-s_st)));
v(find(s<s_st)) = 0;
v(find(s>s_go)) = v_max;
%% V(s)

f1 = figure(1);

plot(s,v_max*ones(size(s)),'--k','linewidth',1);
hold on;
plot(0:1:20,15*ones(length(0:1:20)),'--k','linewidth',1);
plot(20*ones(length(0:1:15)),0:1:15,'--k','linewidth',1);
plot(35*ones(length(0:1:v_max)),0:1:v_max,'--k','linewidth',1);

plot(s,v,'linewidth',2,'Color',[0.0078,0.42,1]);

hold on;

Wsize = 20;  % word size
Tsize = 17;
set(gca,'TickLabelInterpreter','latex','fontsize',Tsize);
grid on;
set(gca,'xtick',0:10:40);
set(gca,'ytick',0:10:30);

text(0.5,30+2,'$v_{\rm max}=30$','fontsize',Tsize,'Interpreter','latex');
text(0.5,15+2,'$v=15$','fontsize',Tsize,'Interpreter','latex');
text(20+0.2,0+2.4,'$s=20$','fontsize',Tsize,'Interpreter','latex');

text(35+0.2,0+2.5,'$s_{\rm go}$','fontsize',Tsize,'Interpreter','latex');
text(5-0.5,0+2.5,'$s_{\rm st}$','fontsize',Tsize,'Interpreter','latex');

xlabel('$s$','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('$V(s)$','fontsize',Wsize,'Interpreter','latex','Color','k');
axis([0 40 0 35]);
set(gcf,'Position',[250 150 400 290]);

fig = gcf;
fig.PaperPositionMode = 'auto';
% print(gcf,['Figs/Fig8a_OVMSpacingPolicy'],'-painters','-depsc2','-r300');



%% \dot{V}(s)
f2 = figure(2);
plot(20*ones(20,1),linspace(0,max(diff(v)),20),'--k','linewidth',1);
hold on;
plot(0:1:20,max(diff(v))*ones(21,1),'--k','linewidth',1);


plot(s(2:end),diff(v),'linewidth',2,'Color',[0.9607,0.1529,0.2196]);

hold on;


set(gca,'TickLabelInterpreter','latex','fontsize',Tsize);
grid on;


xlabel('$s$','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('$\dot{V} (s)$','fontsize',Wsize,'Interpreter','latex','Color','k');
axis([0 40 0 0.2]);
set(gcf,'Position',[250 150 400 290]);
set(gca,'xtick',0:10:40);
set(gca,'ytick',0:0.05:0.2);

text(20+0.2,0+0.014,'$s=20$','fontsize',Tsize,'Interpreter','latex');
text(35+0.2,0+0.014,'$s_{\rm go}$','fontsize',Tsize,'Interpreter','latex');
text(1.6,0+0.014,'$s_{\rm st}$','fontsize',Tsize,'Interpreter','latex');
text(0.2,max(diff(v))+0.014,'${\rm max} \dot{V} (s)$','fontsize',Tsize,'Interpreter','latex');


fig = gcf;
fig.PaperPositionMode = 'auto';
% print(gcf,['Figs/Fig8b_VelocityDot'],'-painters','-depsc2','-r300');
