function [A,B,Q,R] = ring_traffic_model(N,s_star,gamma_s,gamma_v,gamma_u,AV_number)
% Generate a dynamic model for 

    %OVM
%     if nargin < 2
%         OVM = 1;
%     end
    OVM = 1;
    
    alpha  = 0.6;
    beta   = 0.9;
    v_max  = 30;
    s_st   = 5;
    s_go   = 35;
    %s_star = 0.5*(s_st+s_go);

    % General
    if OVM
        alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
        alpha2 = alpha+beta;
        alpha3 = beta;
    else
        alpha1 = 2.6;
        alpha2 = 3;
        alpha3 = 2;
    end


    A1 = [0,-1;alpha1,-alpha2];
    A2 = [0,1;0,alpha3];
    C1 = [0,-1;0,0];
    C2 = [0,1;0,0];


    pos1 = 1;
    pos2 = N;

    %Y = AY+Bu
    A = zeros(2*N,2*N);
   
    for i=1:(N-1)
        A((2*i-1):(2*i),(2*pos1-1):(2*pos1))=A1;
        A((2*i-1):(2*i),(2*pos2-1):(2*pos2))=A2;
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
    
    B = zeros(2*N,AV_number);
    B(2*N,1) = 1;
    if AV_number == 2
        AV2_Index = floor(N/2);
        A((2*AV2_Index-1):(2*AV2_Index),(2*AV2_Index-1):(2*AV2_Index))=C1;
        A((2*AV2_Index-1):(2*AV2_Index),(2*AV2_Index-3):(2*AV2_Index-2))=C2;
        B(2*AV2_Index,2) = 1;
    end
    
    R = gamma_u*eye(AV_number,AV_number);
    %[K,S,e] = lqr(A,B,Q,R,0);
    %[P,L,G] = care(A,B,Q);
end

