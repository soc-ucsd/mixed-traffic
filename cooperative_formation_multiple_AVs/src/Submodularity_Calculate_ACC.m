%% Description
% Calculate the marginal improvement of ACC to check its submodularity
% Correspond to the data source in Fig. 6 in the paper

clc;
clear;
close all;

addpath('..\_fcn');



%% Key Parameter

N = 12; % Number of AVs
TestNumber = 5; % Number of random test cases

%% 

Gain = zeros(TestNumber,N-1);

OVM_bool = 0;
Controller = 3;

% for OVM_bool = 0:1
% for Controller = 2:3


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



switch Controller
    
    case 1
        
        kv = 5;
        ks = 0.5; % smaller than alpha1
                
    case 2
        kv = 3;
        ks = 0.3; % smaller than alpha1
        
    case 3
        kv = 1;
        ks = 0.1; % smaller than alpha1
                
end

gammaType = 1; % The same for different types with the same ratio


%%

h=waitbar(0,'please wait');




for iTest=1:TestNumber
    
    S = zeros(1,N);
   
    initial_ID = randi(N-1)+1;
    S(initial_ID) = 1;
    
    S_plus = S;
    S_plus(1) = 1;
    
    
    J = -ReturnObjectiveValue_ACC(S,N,alpha1,alpha2,alpha3,gammaType,kv,ks);
    J_plus = -ReturnObjectiveValue_ACC(S_plus,N,alpha1,alpha2,alpha3,gammaType,kv,ks);
    
    Gain(iTest,1) = J_plus - J;
    
    for iG=2:(N-1)
        HDV_ID = find(S_plus==0);
        new_AV_ID = HDV_ID(randi(length(HDV_ID)));
        S(new_AV_ID) = 1;
        S_plus(new_AV_ID) = 1;
        J = -ReturnObjectiveValue_ACC(S,N,alpha1,alpha2,alpha3,gammaType,kv,ks);
    J_plus = -ReturnObjectiveValue_ACC(S_plus,N,alpha1,alpha2,alpha3,gammaType,kv,ks);
    Gain(iTest,iG) = J_plus - J;
    end
     
    str=['Processing...',num2str(iTest/TestNumber*100),'%'];
    waitbar(iTest/TestNumber,h,str);
end
close(h);
%% 


% if OVM_bool
% save(['..\_Data\Submodularity\ACC\OVM_',date,'_N_',num2str(N),'_Controller_',num2str(Controller),'.mat']);
% else
%     save(['..\_Data\Submodularity\ACC\NotOVM_',date,'_N_',num2str(N),'_Controller_',num2str(Controller),'.mat']);
% end

for i = 1:TestNumber
    plot(Gain(i,:));
    hold on;
end

% end
% end