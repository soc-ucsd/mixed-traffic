function [K] = lqrsdp(A,B1,B,Q,R)

epsilon   = 1e-5;

%% Call Yalmip

n = size(A,1);  % number of states
m = size(B,2);  % number of inputs


% variables
X = sdpvar(n);
Z = sdpvar(m,n);
Y = sdpvar(m);

% constraints
Constraints = [A*X - B*Z + (A*X - B*Z)' + B1*B1' <=0,
               X - epsilon*eye(n)>=0,
               [Y Z;Z' X] >=0];
           
obj   = trace(Q*X) + trace(R*Y);
%opts  = sdpsettings('solver','sedumi');   % call SDP solver: sedumi
opts  = sdpsettings('solver','mosek');     % call SDP solver: mosek, this is often better

sol     = optimize(Constraints,obj,opts);

Xd = value(X);
Zd = value(Z);

K = Zd*Xd^(-1);

end
