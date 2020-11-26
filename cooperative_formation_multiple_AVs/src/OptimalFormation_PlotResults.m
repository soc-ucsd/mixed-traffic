%% Description
% Plot the optimal formation in mixed traffic flow.
% Correspond to Fig. 10 in our paper.



clc;
clear;
close all;

for SetupI = 1:4
    
    switch SetupI
        case 1
            load('..\_data\Optimal Formation\17-Sep-2019_OptimalPlacement_OVM_N_12_AV_2_gammaType_1.mat');
        case 2
            load('..\_data\Optimal Formation\18-Sep-2019_OptimalPlacement_OVM_N_12_AV_4_gammaType_1.mat');
        case 3
            load('..\_data\Optimal Formation\18-Sep-2019_OptimalPlacement_OVM_N_12_AV_2_gammaType_2.mat');
        case 4
            load('..\_data\Optimal Formation\19-Sep-2019_OptimalPlacement_OVM_N_12_AV_4_gammaType_2.mat');
    end
    
    v_max  = 30;
    s_st   = 5;
    s_go   = 35;
 
    %% process the color
    
    C = zeros(length(Alpha),length(Beta),length(S_star));
    C_Best = C;
    C_Worst = C;
    
    
    for iA = 1:length(Alpha)
        for iB = 1:length(Beta)
            for iS = 1:length(S_star)
                alpha = Alpha(iA);
                beta = Beta(iB);
                s_star = S_star(iS);
                C(iA,iB,iS) = alpha+2*beta-2*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
            end
        end
    end
    c0_min_best = min(C(find(BestResult==0)));
    c0_max_best = max(C(find(BestResult==0)));
    c1_min_best = min(C(find(BestResult==1)));
    c1_max_best = max(C(find(BestResult==1)));
    
    
    
    C_Best(find(BestResult==0)) = (C(find(BestResult==0))-c0_min_best)/(c0_max_best-c0_min_best);
    C_Best(find(BestResult==1)) = 2+(C(find(BestResult==1))-c1_min_best)/(c1_max_best-c1_min_best);
    C_Best(find(C_Best==1)) = 0.99999;
    C_Best(find(C_Best==2)) = 2.00001;
    
    
    
    c0_min_worst = min(C(find(WorstResult==0)));
    c0_max_worst = max(C(find(WorstResult==0)));
    c1_min_worst = min(C(find(WorstResult==1)));
    c1_max_worst = max(C(find(WorstResult==1)));
    
    
    
    C_Worst(find(WorstResult==0)) = (C(find(WorstResult==0))-c0_min_worst)/(c0_max_worst-c0_min_worst);
    C_Worst(find(WorstResult==1)) = 2+(C(find(WorstResult==1))-c1_min_worst)/(c1_max_worst-c1_min_worst);
    C_Worst(find(C_Worst==1)) = 0.99999;
    C_Worst(find(C_Worst==2)) = 2.00001;
    %% plot Best Placement
    figure(1);
     
    load('ColorMap_RedWhiteBlue.mat');
    
    colormap(mymap_red_white_blue);

    
    ScatterSsize = 23;
    FontSize = 18;
    
    for iA = 1:length(Alpha)
        for iB = 1:length(Beta)
            for iS = 1:length(S_star)
                alpha = Alpha(iA);
                beta = Beta(iB);
                s_star = S_star(iS);
                if BestResult(iA,iB,iS)==0
                    scatter3(alpha,beta,s_star,ScatterSsize,C_Best(iA,iB,iS),'o','filled');
                    hold on;
                    scatter3(alpha,beta,40-s_star,ScatterSsize,C_Best(iA,iB,iS),'o','filled');
                elseif BestResult(iA,iB,iS)==1
                    scatter3(alpha,beta,s_star,ScatterSsize,C_Best(iA,iB,iS),'^','filled');
                    hold on;
                    scatter3(alpha,beta,40-s_star,ScatterSsize,C_Best(iA,iB,iS),'^','filled');
                else
                    s = scatter3(alpha,beta,s_star,ScatterSsize+5,C_Best(iA,iB,iS),'p','filled');
                    s.MarkerFaceColor = [0.4,0.4,0.4];
                    hold on;
                    s = scatter3(alpha,beta,40-s_star,ScatterSsize+5,C_Best(iA,iB,iS),'p','filled');
                    s.MarkerFaceColor = [0.4,0.4,0.4];
                end
            end
        end
    end
    
    
    
    
    
    
    
    set(gca,'TickLabelInterpreter','latex','fontsize',14);
    grid off;
    set(gca,'zlim',[5,35]);
    
    xl = xlabel('$\alpha$','fontsize',FontSize,'Interpreter','latex','Color','k');
    yl = ylabel('$\beta$','fontsize',FontSize,'Interpreter','latex','Color','k');
    zl = zlabel('$s^*$','fontsize',FontSize,'Interpreter','latex','Color','k');
    
    xl.Position = [0.75,2.0351,2.5];    
    yl.Position = [-0.1655,1,4];
    zl.Position = [1.645,2.0367,20];
    
    
    title(['Optimal Formation'],'Interpreter','latex','FontSize',FontSize+2,'Color','k');
    
    
    set(gcf,'Position',[250 150 410 390]);
    fig = gcf;
    fig.PaperPositionMode = 'auto';
    
    view(-172,10);
    
    %% Plot Worst Result
    
    figure(2);
    
    load('ColorMap_RedWhiteBlue.mat');
    mymap_red_white_blue_reverse = mymap_red_white_blue;
    for i=1:36
        mymap_red_white_blue_reverse(i,:) = mymap_red_white_blue(37-i,:);
        mymap_red_white_blue_reverse(i+72,:) = mymap_red_white_blue(109-i,:);
    end
    
    colormap(mymap_red_white_blue_reverse);
    
   
    for iA = 1:length(Alpha)
        for iB = 1:length(Beta)
            for iS = 1:length(S_star)
                alpha = Alpha(iA);
                beta = Beta(iB);
                s_star = S_star(iS);
                if WorstResult(iA,iB,iS)==0
                    scatter3(alpha,beta,s_star,ScatterSsize-10,C_Worst(iA,iB,iS),'o','LineWidth',0.8);
                    hold on;
                    scatter3(alpha,beta,40-s_star,ScatterSsize-10,C_Worst(iA,iB,iS),'o','LineWidth',0.8);
                elseif WorstResult(iA,iB,iS)==1
                    scatter3(alpha,beta,s_star,ScatterSsize-10,C_Worst(iA,iB,iS),'^','LineWidth',0.8);
                    hold on;
                    scatter3(alpha,beta,40-s_star,ScatterSsize-10,C_Worst(iA,iB,iS),'^','LineWidth',0.8);
                else
                    s = scatter3(alpha,beta,s_star,ScatterSsize-5,C_Worst(iA,iB,iS),'p','LineWidth',0.8);
                    s.MarkerEdgeColor = [0.4,0.4,0.4];
                    hold on;
                    s = scatter3(alpha,beta,40-s_star,ScatterSsize-5,C_Worst(iA,iB,iS),'p','LineWidth',0.8);
                    s.MarkerEdgeColor = [0.4,0.4,0.4];
                end
            end
        end
    end
    
    
    set(gca,'TickLabelInterpreter','latex','fontsize',14);
    set(gca,'zlim',[5,35]);
    
    grid off;
    
    xl = xlabel('$\alpha$','fontsize',FontSize,'Interpreter','latex','Color','k');
    yl = ylabel('$\beta$','fontsize',FontSize,'Interpreter','latex','Color','k');
    zl = zlabel('$s^*$','fontsize',FontSize,'Interpreter','latex','Color','k');
    
    xl.Position = [0.75,2.0351,2.5];    
    yl.Position = [-0.1655,1,4];
    zl.Position = [1.645,2.0367,20];
    
    title(['Worst Formation'],'Interpreter','latex','FontSize',FontSize+2,'Color','k');
    
    
    set(gcf,'Position',[250 150 410 390]);
    view(-172,10);
    
    fig = gcf;
    fig.PaperPositionMode = 'auto';
    
    
    
%     switch SetupI
%         case 1
%             figure(1);
%             print(gcf,['Figs/Fig5_OptimalFormation1'],'-painters','-depsc2','-r300');
%             figure(2);
%             print(gcf,['Figs/Fig5_WorstFormation1'],'-painters','-depsc2','-r300');
%         case 2
%             figure(1);
%             print(gcf,['Figs/Fig5_OptimalFormation2'],'-painters','-depsc2','-r300');
%             figure(2);
%             print(gcf,['Figs/Fig5_WorstFormation2'],'-painters','-depsc2','-r300');
%         case 3
%             figure(1);
%             print(gcf,['Figs/Fig5_OptimalFormation3'],'-painters','-depsc2','-r300');
%             figure(2);
%             print(gcf,['Figs/Fig5_WorstFormation3'],'-painters','-depsc2','-r300');
%         case 4
%             figure(1);
%             print(gcf,['Figs/Fig5_OptimalFormation4'],'-painters','-depsc2','-r300');
%             figure(2);
%             print(gcf,['Figs/Fig5_WorstFormation4'],'-painters','-depsc2','-r300');   
%     end
%     close all;
        
end

