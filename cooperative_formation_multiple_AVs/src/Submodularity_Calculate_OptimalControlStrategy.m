%% Description
% Calculate the marginal improvement of optimal control strategy (OCS) to check its submodularity
% Correspond to the data source in Fig. 7 in the paper

clc;
clear;
close all;

addpath('..\_fcn');

%% Key Parameters

N = 12; % Number of AVs
TestNumber = 5; % Number of random test cases
gammaType = 9999; % Setup of weight coefficients in OCS
% 9999 represents a sufficiently small punishment on the control input

%% 

Gain = zeros(TestNumber,N-1);
GainID = zeros(TestNumber,N-1);




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

%%

h=waitbar(0,'please wait');


for iTest=1:TestNumber
    
    S = zeros(1,N);
   
    initial_ID = randi(N-1)+1;
    GainID(iTest,1) = initial_ID;
    
    S(initial_ID) = 1;
    
    S_plus = S;
    S_plus(1) = 1;
    
    
    J = -ReturnObjectiveValue(S,N,alpha1,alpha2,alpha3,gammaType);
    J_plus = -ReturnObjectiveValue(S_plus,N,alpha1,alpha2,alpha3,gammaType);
    
    Gain(iTest,1) = J_plus - J;
    
    for iG=2:(N-1)
        HDV_ID = find(S_plus==0);
        new_AV_ID = HDV_ID(randi(length(HDV_ID)));
        GainID (iTest,iG) = new_AV_ID;
        S(new_AV_ID) = 1;
        S_plus(new_AV_ID) = 1;
        J = -ReturnObjectiveValue(S,N,alpha1,alpha2,alpha3,gammaType);
    J_plus = -ReturnObjectiveValue(S_plus,N,alpha1,alpha2,alpha3,gammaType);
    Gain(iTest,iG) = J_plus - J;
    end
     
    str=['Processing...',num2str(iTest/TestNumber*100),'%'];
    waitbar(iTest/TestNumber,h,str);
end
close(h);
%% 

% switch OVM_bool
%     case 1
% save(['..\_Data\Submodularity\ACC\Optimal Control Strategy\OVM_',date,'_N_',num2str(N),'_GammaType_',num2str(gammaType),'.mat']);
%     case 0
%         save(['..\_Data\Submodularity\ACC\Optimal Control Strategy\NotOVM_',date,'_N_',num2str(N),'_GammaType_',num2str(gammaType),'.mat']);
% end

for i = 1:TestNumber
    plot(Gain(i,:));
    hold on;
end