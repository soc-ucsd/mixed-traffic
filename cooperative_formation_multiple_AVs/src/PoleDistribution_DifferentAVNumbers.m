%% Description
% Plot the distribution of closed-loop poles at different numbers of AVs

clc;
clear;
close all;

%% Parameter

N = 20;

OVM_bool = 1;

if OVM_bool
    alpha  = 0.6;
    beta   = 0.9;
    s_star = 20;
    v_max  = 30;
    s_st   = 5;
    s_go   = 35;
    alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
    alpha2 = alpha+beta;
    alpha3 = beta;
else
    alpha1 = 0.5;
    alpha2 = 2.5;
    alpha3 = 0.5;
end


% Extra feedback gain in AVs
ks = 0.1;
kv = 1;

% A1,A2 are for HDVs
% C1,C2 are for AVs
A1 = [0,-1;alpha1,-alpha2];
A2 = [0,1;0,alpha3];
C1 = [0,-1;alpha1-ks,-alpha2-kv];
C2 = [0,1;0,alpha3];

% Closed-loop model
A = zeros(2*N,2*N);

A(1:2,1:2) = A1;
A(1:2,(2*N-1):2*N) = A2;
for i=2:N
    A((2*i-1):(2*i),(2*i-1):(2*i))=A1;
    A((2*i-1):(2*i),(2*i-3):(2*i-2))=A2;
end



%% Pole Distribution at Different Numbers of AVs



Pole_Color = [0.0078,0.42,1];
Background_Color = [255, 243, 242]/255;


Marker_Size = 14;

Wsize = 16;

figure;


for k=0:2:N
    
    if k ==0
        
    elseif k == 1
        A(1:2,1:2) = C1;
        A(1:2,(2*N-1):2*N) = C2;
    else % At each time of the loop, turn two more HDVs into AVs
        A((2*k-1):(2*k),(2*k-1):(2*k))=C1;
        A((2*k-1):(2*k),(2*k-3):(2*k-2))=C2;
    end
    eigenvalue = eig(A);
    %Scatter all eigenvalues
    scatter3(k*ones(2*N,1)-0.03,real(eigenvalue),imag(eigenvalue),Marker_Size,Pole_Color,'o','filled');
    hold on;
    v1 = [k,0.5,2.0;k,-3,2.0;k,-3,-2.0;k,0.5,-2.0];
    f1 = [1 2 3 4];
    patch('faces',f1,'vertices',v1,'facecolor',Background_Color,'FaceAlpha',1,'EdgeColor','none');
    
    
    
end


set(gca,'xlim',[-1,N+1]);
set(gca,'zlim',[-1.5,1.5]);
set(gca,'ylim',[-2.5,0]);

set(gca,'ztick',-1.5:0.75:1.5);

ax = gca;
grid on;


set(gca,'TickLabelInterpreter','latex','fontsize',12);

switch N
    case 20
        set(gcf,'Position',[150 150 1200 280]);
        view([-4 11]);
    case 12
        set(gcf,'Position',[150 150 500 330]);
        view([-4 3]);
end

xl = xlabel('$|S|$','fontsize',Wsize,'Interpreter','latex','Color','k');
yl = ylabel('$\mathrm{Re}(\lambda)$','fontsize',Wsize,'Interpreter','latex','Color','k');
zl = zlabel('$\mathrm{Im}(\lambda)$','fontsize',Wsize,'Interpreter','latex','Color','k');

xl.Position = [21.5,-2,-1.5];
yl.Position=[-3.5,-1,-1.5];
zl.Position = [-2.7,0,0];



fig = gcf;
fig.PaperPositionMode = 'auto';
% print(gcf,'Figs/Fig3_PoleDistribution','-depsc','-r300');
