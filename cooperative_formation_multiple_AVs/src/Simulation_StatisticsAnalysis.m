%% Description
% Calculate two performance metrics in differnet positions of the perturbation
% Time to stabilize and linear quadratic cost
% Simulation_BrakeScenario needs to run first to generate the necessary data
% Correspond to Fig. 13 in our paper.


clc;
close all;
clear;

%% Key parameters

%%%%%%%%%%%%%%%%
platoon_bool = 0; % 0 or 1
%%%%%%%%%%%%%%%%


N = 20;
AV_number = 4;

statistics_bool = 1;


%% Statistics Calculation

if statistics_bool
    ConvergenceTime = zeros(N,2);
    ControlEnergy = zeros(N,2);
    
    smoothTimeLength = 10;  % smooth the control input for every 0.2 seconds
    settlingTimeThreshold = 0.75;
    LQCost = zeros(N,2);
    for platoon_bool = 0:1
        for brakeID = 1:20
            switch platoon_bool
                %%%%%%%%%%%%%%%%%%
                % Change the data source
                case 1
                    load(['..\_data\Traffic Simulation\Platoon_BrakeID_',num2str(brakeID),'_gammaType_2_N_',num2str(20),'_AV_',num2str(AV_number),'.mat']);
                case 0
                    load(['..\_data\Traffic Simulation\Uniform_BrakeID_',num2str(brakeID),'_gammaType_2_N_',num2str(20),'_AV_',num2str(AV_number),'.mat']);
                %%%%%%%%%%%%%%%%%%%
            end
            
            k = NumStep;
            while 1
                v_minus = S(k,:,2)-V_avg(k);
                if isempty(find(v_minus>settlingTimeThreshold,1))
                    k = k-1;
                else
                    break;
                end
            end
            ConvergenceTime(brakeID,platoon_bool+1) = k*Tstep-20;
            S(:,N,3) = smooth(S(:,N,3),smoothTimeLength);
            ControlEnergy(brakeID,platoon_bool+1) = sum(S(:,N,3).^2)*Tstep;
            
            
            switch gammaType
                
                case 1
                    %S1
                    gamma_s = 0.01;
                    gamma_v = 0.05;
                    gamma_u = 0.1;
                    
                case 2
                    %S2
                    gamma_s = 0.03;
                    gamma_v = 0.15;
                    gamma_u = 0.1;
                    
                case 3
                    %S3
                    gamma_s = 0.05;
                    gamma_v = 0.25;
                    gamma_u = 0.1;
                case 4
                    gamma_s = 0.03;
                    gamma_v = 0.15;
                    gamma_u = 1;
                case 5
                    gamma_s = 1;
                    gamma_v = 1;
                    gamma_u = 0;
                    
            end
            Q = zeros(2*N);
            for i=1:N
                Q(2*i-1,2*i-1) = gamma_s;
                Q(2*i,2*i) = gamma_v;
            end
            R = gamma_u*eye(AV_number);
            
            X = zeros(TotalTime/Tstep,N);
            for k=1:NumStep
                for i=1:N
                    if i==1
                        X(k,i) = S(k,N,1)+Circumference-S(k,i,1)-s_star;
                        X(i,i+1) = S(k,i,2)-v_star;
                    else
                        X(k,2*i-1) = S(k,i-1,1)-S(k,i,1)-s_star;
                        X(k,2*i) = S(k,i,2)-v_star;
                    end
                end
                
                Input = zeros(1,AV_number);
                iInput = 1;
                for i=1:N
                    if ID(i) == 1
                        Input(iInput) = S(k,i,3);
                        iInput = iInput+1;
                    end
                end
                
                LQCost(brakeID,platoon_bool+1) = LQCost(brakeID,platoon_bool+1)+X(k,:)*Q*X(k,:)'+Input*R*Input';
            end
        end
    end
end
%% Plot Statistics

if statistics_bool
    Lname1 = 'Uniform distribution';
    Lname2 = 'Platoon formation';
    Wsize = 18;
    LWidth = 1.5;
    MSize  = 8;
    
    % Convergence Time
    
    figure;
    switch AV_number
        case 4
            for id=3:5:18
                x = [id-0.4,id+0.4,id+0.4,id-0.4];
                y = [0,0,ConvergenceTime(id,1),ConvergenceTime(id,1)];
                p = patch(x,y,'k');
                p.FaceColor = [0.9764,0.7490,0.7725];
                p.EdgeColor = 'none';
                hold on;
                
                
            end
            for id=9:1:12
                x = [id-0.4,id+0.4,id+0.4,id-0.4];
                y = [0,0,ConvergenceTime(id,2),ConvergenceTime(id,2)];
                p = patch(x,y,'k');
                p.FaceColor = [0.6627,0.8392,0.9961];
                p.EdgeColor = 'none';
                hold on;
            end
        case 2
            for id=5:10:15
                x = [id-0.4,id+0.4,id+0.4,id-0.4];
                y = [0,0,ConvergenceTime(id,1),ConvergenceTime(id,1)];
                p = patch(x,y,'k');
                p.FaceColor = [0.9764,0.7490,0.7725];
                p.EdgeColor = 'none';
                hold on;
                
                
            end
            for id=10:1:11
                x = [id-0.4,id+0.4,id+0.4,id-0.4];
                y = [0,0,ConvergenceTime(id,2),ConvergenceTime(id,2)];
                p = patch(x,y,'k');
                p.FaceColor = [0.6627,0.8392,0.9961];
                p.EdgeColor = 'none';
                hold on;
            end
    end
    
    plot(1:20,ConvergenceTime(:,1),'Color',[0.9607,0.1529,0.2196],'linewidth',LWidth); hold on;
    p1 = plot(1:20,ConvergenceTime(:,1),'o','Color',[0.9607,0.1529,0.2196],'linewidth',1.5,'markersize',MSize,'displayname',Lname1);
    
    plot(1:20,ConvergenceTime(:,2),'Color',[0.0078,0.42,1],'linewidth',LWidth); hold on;
    p2 = plot(1:20,ConvergenceTime(:,2),'*','Color',[0.0078,0.42,1],'linewidth',1.5,'markersize',MSize,'displayname',Lname2);
    
    
    switch AV_number
        case 4
            set(gca,'ylim',[14 30]);
        case 2
            set(gca,'ylim',[15 33]);
    end
    
    set(gca,'xlim',[1 20]);
    set(gca,'ylim',[14 32]);
    set(gca,'TickLabelInterpreter','latex','fontsize',16);
    set(gca,'xtick',[0:5:20]);
    set(gca,'ytick',[14:4:32]);
    
    xlabel('Position of the perturbation','fontsize',Wsize,'Interpreter','latex','Color','k');
    ylabel('Time to stabilize ($\mathrm{s}$)','fontsize',Wsize,'Interpreter','latex','Color','k');
    
    
    grid on;box on;
    l = legend([p1 p2]);
    l.Location = 'NorthEast';
    l.Interpreter = 'latex';
    l.FontSize = Wsize;
    l.Box = 'off';
    
    set(gcf,'Position',[250 150 550 320]);
    fig = gcf;
    fig.PaperPositionMode = 'auto';
    %     print(gcf,'Figs/Fig14_Time','-depsc','-r300');
    
    %LQ cost
    figure;
    switch AV_number
        case 4
            for id=3:5:18
                x = [id-0.4,id+0.4,id+0.4,id-0.4];
                y = [0,0,LQCost(id,1),LQCost(id,1)];
                p = patch(x,y,'k');
                p.FaceColor = [0.9764,0.7490,0.7725];
                p.EdgeColor = 'none';
                hold on;
                
            end
            for id=9:1:12
                x = [id-0.4,id+0.4,id+0.4,id-0.4];
                y = [0,0,LQCost(id,2),LQCost(id,2)];
                p = patch(x,y,'k');
                p.FaceColor = [0.6627,0.8392,0.9961];
                p.EdgeColor = 'none';
                hold on;
            end
        case 2
            for id=5:10:15
                x = [id-0.4,id+0.4,id+0.4,id-0.4];
                y = [0,0,LQCost(id,1),LQCost(id,1)];
                p = patch(x,y,'k');
                p.FaceColor = [0.9764,0.7490,0.7725];
                p.EdgeColor = 'none';
                hold on;
                
            end
            for id=10:1:11
                x = [id-0.4,id+0.4,id+0.4,id-0.4];
                y = [0,0,LQCost(id,2),LQCost(id,2)];
                p = patch(x,y,'k');
                p.FaceColor = [0.6627,0.8392,0.9961];
                p.EdgeColor = 'none';
                hold on;
            end
    end
    
    
    plot(1:20,LQCost(:,1),'Color',[0.9607,0.1529,0.2196],'linewidth',LWidth); hold on;
    p1 = plot(1:20,LQCost(:,1),'o','Color',[0.9607,0.1529,0.2196],'linewidth',1.5,'markersize',MSize,'displayname',Lname1);
    
    plot(1:20,LQCost(:,2),'Color',[0.0078,0.42,1],'linewidth',LWidth); hold on;
    p2 = plot(1:20,LQCost(:,2),'*','Color',[0.0078,0.42,1],'linewidth',1.5,'markersize',MSize,'displayname',Lname2);
    
    
    switch AV_number
        case 4
            set(gca,'ylim',[3e4 12e4]);
        case 2
            set(gca,'ylim',[4e4 14e4]);
    end
    
    set(gca,'xlim',[1 20]);
    set(gca,'TickLabelInterpreter','latex','fontsize',16);
    
    xlabel('Position of the perturbation','fontsize',Wsize,'Interpreter','latex','Color','k');
    ylabel('Linear Quadratic Cost','fontsize',Wsize,'Interpreter','latex','Color','k');
    
    grid on;box on;
    l = legend([p1 p2]);
    l.Location = 'NorthEast';
    l.Interpreter = 'latex';
    l.FontSize = Wsize;
    l.Box = 'off';
    
    set(gcf,'Position',[250 150 550 320]);
    fig = gcf;
    fig.PaperPositionMode = 'auto';
    
    %     print(gcf,'Figs/Fig14_Cost','-depsc','-r300');
    
end