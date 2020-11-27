function [K_Opt,Info] = optsi(A,B1,B2,K_Pattern,Q,R)
% For a given pattern of K, calculate the optimal feedback gain using
% sparsity invirance
% A: system matrix;  B1: distrubance matrix;  B2: control input matrix;

n = size(A,1);
m = size(B2,2); % number of driver nodes
epsilon = 1e-5;

%% Sparsity invariance
Tp = K_Pattern;
Rp = pattern_invariance(Tp);

%% variables
X = sdpvar(n);               %% block diagonal X
for i = 1:n
    for j = i:n
       if Rp(i,j) == 0
           X(i,j) = 0; X(j,i) = 0;
       end
    end
end

Z = sdpvar(m,n);        %% Matrix Z has sparsity pattern in K_Pattern
for i = 1:m
    for j = 1:n
       if Tp(i,j) == 0
           Z(i,j) = 0;
       end
    end
end
Y = sdpvar(m);
%% constraint

Const = [X-epsilon*eye(n) >= 0, [Y Z; Z' X] >= 0,... 
    (A*X-B2*Z)+(A*X-B2*Z)'+B1*B1' <= 0];

%% cost function

Obj = trace(Q*X)+trace(R*Y);

%% solution via mosek
ops = sdpsettings('solver','mosek');
Info = optimize(Const,Obj,ops);
X1 = value(X);
Z1 = value(Z);
K_Opt = Z1*X1^(-1);


end

