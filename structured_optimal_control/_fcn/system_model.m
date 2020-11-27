function [A,B1,B2,Q,R] = system_model(N,AV_number,alpha,beta,v_max,s_st,s_go,s_star,gamma_s,gamma_v,gamma_u)


alpha1 = alpha.*v_max/2*pi./(s_go-s_st).*sin(pi*(s_star-s_st)./(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;

C1 = [0,-1;0,0];
C2 = [0,1;0,0];


pos1 = 1;
pos2 = N;

%Y = AY+Bu
A = zeros(2*N,2*N);

for i=1:(N-1)
    A((2*i-1):(2*i),(2*pos1-1):(2*pos1))=[0,-1;alpha1(i),-alpha2(i)];
    A((2*i-1):(2*i),(2*pos2-1):(2*pos2))=[0,1;0,alpha3(i)];
    pos1 = pos1+1;
    pos2 = mod(pos2+1,N);
end
A((2*N-1):(2*N),(2*pos1-1):(2*pos1))=C1;
A((2*N-1):(2*N),(2*pos2-1):(2*pos2))=C2;
%Controller

Q = zeros(2*N);
for i=1:N
    Q(2*i-1,2*i-1) = gamma_s;
    Q(2*i,2*i) = gamma_v;
end

B2 = zeros(2*N,AV_number);
B2(2*N,AV_number) = 1;
if AV_number == 2
    AV2_Index = floor(N/2);
    A((2*AV2_Index-1):(2*AV2_Index),(2*AV2_Index-1):(2*AV2_Index))=C1;
    A((2*AV2_Index-1):(2*AV2_Index),(2*AV2_Index-3):(2*AV2_Index-2))=C2;
    B2(2*AV2_Index,1) = 1;
end

B1 = zeros(2*N,N);
for i=1:N
    B1(2*i,i) = 1;
end

R = gamma_u*eye(AV_number,AV_number);


end

