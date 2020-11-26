%% Description
% Find the optimal formation and worst formation by enumeration.
% Correspond to the data source in Fig. 10 in our paper.

clc;
clear;
close all;

addpath('..\_fcn');
%% Major parameters

N = 12; % total number of vehicles
AV_number = 4; % number of AVs

gammaType = 1; % weight coefficient matrices in optimal control

%% 

%%%%%%%%%%%%%%%%%%
% Change resolution
Alpha = 0.1:0.2:1.5;
Beta = 0.1:0.2:1.5;
S_star = 6:2:20;
%%%%%%%%%%%%%%%%%%

v_max  = 30;
s_st   = 5;
s_go   = 35;


BestResult = zeros(length(Alpha),length(Beta),length(S_star));
% 1.Distribution Type 2-4. alpha1,alpha2,alpha3
WorstResult = zeros(length(Alpha),length(Beta),length(S_star));



h=waitbar(0,'please wait');
% ExpeNumber = length(Alpha)*length(Beta)*length(S_star);
ParExpeNumber = length(Alpha)*length(Beta);
iExpe = 1;
%% 

for iA = 1:length(Alpha)
    for iB = 1:length(Beta)
        parfor iS = 1:length(S_star)
            alpha = Alpha(iA);
            beta = Beta(iB);
            s_star = S_star(iS);
            
            alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
            alpha2 = alpha+beta;
            alpha3 = beta;

    
    switch AV_number
        case 1
            Obj_Value = zeros(N,1);
        case 2
            Obj_Value = zeros(N,N);
        case 3
            Obj_Value = zeros(N,N,N);
        case 4
            Obj_Value = zeros(N,N,N,N);
        case 5
            Obj_Value = zeros(N,N,N,N,N);
    end
    
    Obj_Value = zeros(N);
    
  
    
    %% Loop
    switch AV_number
        case 1
            for id1=1:N
                Best_ID = zeros(N,1);
                Best_ID(id1) = 1;
                Obj_Value(id1) = ReturnObjectiveValue(Best_ID,N,alpha1,alpha2,alpha3,gammaType);
            end
        case 2
            id1 = 1;
            cal_i=1;
            for id2 = id1:N
                if id2~=id1
                    % AV Distribution Pattern
%                     Best_ID = zeros(N,1);
                    Best_ID(id1) = 1;
                    Best_ID(id2) = 1;
                    Obj_Value(id1,id2) = ReturnObjectiveValue(Best_ID,N,alpha1,alpha2,alpha3,gammaType);  
                end
            end
            
        case 3
            id1 = 1;
            cal_i=1;
            for id2 = (id1+1):(N-1)
                for id3 = (id2+1):(N)
                    
                    
                    % AV Distribution Pattern
                    Best_ID = zeros(N,1);
                    Best_ID(id1) = 1;
                    Best_ID(id2) = 1;
                    Best_ID(id3) = 1;
                    
                    Obj_Value(id1,id2,id3) = ReturnObjectiveValue(Best_ID,N,alpha1,alpha2,alpha3,gammaType);
                    
                end
                
            end
        case 4
            id1 = 1;
            cal_i=1;
            for id2 = (id1+1):(N-2)
                for id3 = (id2+1):(N-1)
                    for id4 = (id3+1):N
                        
                        % AV Distribution Pattern
                        Best_ID = zeros(N,1);
                        Best_ID(id1) = 1;
                        Best_ID(id2) = 1;
                        Best_ID(id3) = 1;
                        Best_ID(id4) = 1;
                        Obj_Value(id1,id2,id3,id4) = ReturnObjectiveValue(Best_ID,N,alpha1,alpha2,alpha3,gammaType);
                        
                    end
                end
                
            end
            
        case 5
            id1 =1;
            cal_i=1;
            for id2 = (id1+1):(N-3)
                for id3 = (id2+1):(N-2)
                    for id4 = (id3+1):(N-1)
                        for id5 = (id4+1):N
                            % AV Distribution Pattern
                            Best_ID = zeros(N,1);
                            Best_ID(id1) = 1;
                            Best_ID(id2) = 1;
                            Best_ID(id3) = 1;
                            Best_ID(id4) = 1;
                            Best_ID(id5) = 1;
                            Obj_Value(id1,id2,id3,id4,id5) = ReturnObjectiveValue(Best_ID,N,alpha1,alpha2,alpha3,gammaType);
                            
                        end
                    end
                end
                
            end
            
    end
    
    
    Obj_Value(find(Obj_Value==0))=NaN;
    switch AV_number
        case 2
            [id1,id2,id3,id4,id5]=ind2sub(size(Obj_Value),find(Obj_Value==min(min(Obj_Value)),1));
            Best_ID = zeros(N,1);
            Best_ID(id1) = 1;
            Best_ID(id2) = 1;
            [id1,id2,id3,id4,id5]=ind2sub(size(Obj_Value),find(Obj_Value==max(max(Obj_Value)),1));
            Wosrt_ID = zeros(N,1);
            Wosrt_ID(id1) = 1;
            Wosrt_ID(id2) = 1;
        case 3
            [id1,id2,id3,id4,id5]=ind2sub(size(Obj_Value),find(Obj_Value==min(min(min(Obj_Value))),1));
            Best_ID = zeros(N,1);
            Best_ID(id1) = 1;
            Best_ID(id2) = 1;
            Best_ID(id3) = 1;
            [id1,id2,id3,id4,id5]=ind2sub(size(Obj_Value),find(Obj_Value==max(max(max(Obj_Value))),1));
            Wosrt_ID = zeros(N,1);
            Wosrt_ID(id1) = 1;
            Wosrt_ID(id2) = 1;
            Wosrt_ID(id3) = 1;
        case 4
            [id1,id2,id3,id4,id5]=ind2sub(size(Obj_Value),find(Obj_Value==min(min(min(min(Obj_Value)))),1));
            Best_ID = zeros(N,1);
            Best_ID(id1) = 1;
            Best_ID(id2) = 1;
            Best_ID(id3) = 1;
            Best_ID(id4) = 1;
            [id1,id2,id3,id4,id5]=ind2sub(size(Obj_Value),find(Obj_Value==max(max(max(max(Obj_Value)))),1));
            Wosrt_ID = zeros(N,1);
            Wosrt_ID(id1) = 1;
            Wosrt_ID(id2) = 1;
            Wosrt_ID(id3) = 1;
            Wosrt_ID(id4) = 1;
        case 5
            [id1,id2,id3,id4,id5]=ind2sub(size(Obj_Value),find(Obj_Value==min(min(min(min(min(Obj_Value))))),1));
            Best_ID = zeros(N,1);
            Best_ID(id1) = 1;
            Best_ID(id2) = 1;
            Best_ID(id3) = 1;
            Best_ID(id4) = 1;
            Best_ID(id5) = 1;
            [id1,id2,id3,id4,id5]=ind2sub(size(Obj_Value),find(Obj_Value==max(max(max(max(max(Obj_Value))))),1));
            Wosrt_ID = zeros(N,1);
            Wosrt_ID(id1) = 1;
            Wosrt_ID(id2) = 1;
            Wosrt_ID(id3) = 1;
            Wosrt_ID(id4) = 1;
            Wosrt_ID(id5) = 1;
    end
   
    p = find(Best_ID'==1);
    % First check whether uniform distribution
    p_uniform = 1:N/AV_number:(1+N/AV_number*(AV_number-1));
    if all(p==p_uniform)
        type = 0;
    else
        % Then check whether platoon
        p(find(p>N/2))=p(find(p>N/2))-N;
        p1 = sort(p);
        if p1(end)-p1(1)==AV_number-1
            type = 1;
        else
            type = -1;
        end
    end
    BestResult(iA,iB,iS)=type;
    
    p = find(Wosrt_ID'==1);
    % First check whether uniform distribution
    p_uniform = 1:N/AV_number:(1+N/AV_number*(AV_number-1));
    if all(p==p_uniform)
        type = 0;
    else
        % Then check whether platoon
        p(find(p>N/2))=p(find(p>N/2))-N;
        p1 = sort(p);
        if p1(end)-p1(1)==AV_number-1
            type = 1;
        else
            type = -1;
        end
    end
    WorstResult(iA,iB,iS)=type;
    
    
end
% str=['alpha=',num2str(alpha),'; beta = ',num2str(beta),'; Processing...',num2str(iExpe/ParExpeNumber*100),'%'];
    waitbar(iExpe/ParExpeNumber,h);
    iExpe = iExpe+1;
end

end
close(h);
% save(['..\Data\Optimal Formation\',date,'_OptimalPlacement_OVM_N_',num2str(N),'_AV_',num2str(AV_number),'_gammaType_',num2str(gammaType),'.mat']);
