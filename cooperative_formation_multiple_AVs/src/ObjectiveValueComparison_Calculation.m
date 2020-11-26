%% Description
% Calculate the objective value of platoon formation...
% and uniform distribution at different system scales.
% Correspond to the data source in Fig. 11 in our paper.

clc;
clear;
close all;

addpath('..\_fcn');

%% Key parameters

N_collected = 8:4:40;

AV_number = 2;
gammaType = 1;

%% OVM parameters

alpha = 0.6;
beta = 0.9;
s_star = 20;
v_max  = 30;
s_st   = 5;
s_go   = 35;

alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;

%% Calculate the objective value at different system sizes

ObjectiveValue = zeros(length(N_collected),2);

h=waitbar(0,'please wait');

for iN = 1:length(N_collected)
    N = N_collected(iN);
    
    ID_Uniform = zeros(N,1);
    ID_Platoon = zeros(N,1);
    
    for id = 1:AV_number
        
        ID_Uniform(1+N/AV_number*(id-1)) = 1;
        ID_Platoon(id) = 1;
        
    end
    
    ObjectiveValue(iN,1) = ReturnObjectiveValue(ID_Uniform,N,alpha1,alpha2,alpha3,gammaType);
    ObjectiveValue(iN,2) = ReturnObjectiveValue(ID_Platoon,N,alpha1,alpha2,alpha3,gammaType);
    
    str=['Processing...',num2str(iN/length(N_collected)*100),'%'];
    waitbar(iN/length(N_collected),h,str);
end

close(h);
%save(['..\Data\Objective Value Comparison\',date,'_Comparison_OVM_AVnumber_',num2str(AV_number),'_gammaType_',num2str(gammaType),'.mat']);
