%% Description
% Plot the distribution of closed-loop poles at a given case

clc;
clear;
close all;

%% Parameter

N = 12;

OVM_bool = 1;



if OVM_bool
    %%%%% Please Change %%%%%
    alpha  = 0.6;
    beta   = 0.9;
    s_star = 20;
    %%%%% No change %%%%%
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


%%

S = zeros(N,1);
% Spatial Position of the AVs
% 1. AV;  0. HDV
S(1) = 1;
S(5) = 1;
S(6) = 1;
S(3) = 1;


AV_number = length(find(S==1));

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



k=1;
for i=1:N
    if S(i) == 1
        if i == 1
            A(1:2,1:2) = C1;
            A(1:2,(2*N-1):2*N) = C2;
        else
            A((2*i-1):(2*i),(2*i-1):(2*i))=C1;
            A((2*i-1):(2*i),(2*i-3):(2*i-2))=C2;
        end
        B(2*i,k) = 1;
        k = k+1;
    end
end



if isempty(find(real(eig(A))>0.001,1))
    closed_loop_stability = true;
else
    closed_loop_stability = false;
end

pole = eig(A);


%%

Wsize = 20;
LWidth = 2;
MSize  = 8;

scatter(real(pole),imag(pole),30,[0.0078,0.42,1],'filled');

set(gca,'TickLabelInterpreter','latex','fontsize',16);

set(gca,'xlim',[-3,1]);
set(gca,'ylim',[-1.5,1.5]);

xlabel('$\mathrm{Re}(\lambda)$','fontsize',Wsize,'Interpreter','latex','Color','k');
ylabel('$\mathrm{Im}(\lambda)$','fontsize',Wsize,'Interpreter','latex','Color','k');

set(gcf,'Position',[250,150,400,300]);
fig = gcf;
fig.PaperPositionMode = 'auto';


