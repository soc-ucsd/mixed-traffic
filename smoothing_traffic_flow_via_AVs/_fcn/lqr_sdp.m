function [K] = lqr_sdp(N,s_star,gamma_s,gamma_v,gamma_u,AV_number)

    
disp(['N = ',num2str(N)]);

epsilon   = 1e-5;

[A,B,Q,R] = ring_traffic_model(N,s_star,gamma_s,gamma_v,gamma_u,AV_number);   % Dynamics & Performance index


%% Call Yalmip

n = size(A,1);  % number of states
m = size(B,2);  % number of inputs

% assume each vehicle has a deviation
B1 = eye(n);
B1(1:2:n,1:2:n) = 0;
% B1 = B;
% assume disturbance is the same as inout

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
