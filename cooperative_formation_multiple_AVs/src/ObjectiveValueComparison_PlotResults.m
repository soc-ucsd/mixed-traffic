%% Description
% Plot the results of comparison between platoon formation...
% and uniform distribution at different system scales.
% Correspond to Fig. 11 in our paper.


clc;
close all;
clear;

%% 

for  SetupI = 1:4
    
    
    switch SetupI
        case 1
            AV_number = 2;
            gammaType = 1;
        case 2
            AV_number = 4;
            gammaType = 1;
        case 3
            AV_number = 2;
            gammaType = 2;
        case 4
            AV_number = 4;
            gammaType = 2;
    end
       
    
    load(['..\_data\Objective Value Comparison\18-Sep-2019_Comparison_OVM_AVnumber_',num2str(AV_number),'_gammaType_',num2str(gammaType),'.mat']);
    
    
    %% plot figure
    
    
    figure_bool = 1;
    
    if figure_bool
        Lname1 = 'Uniform distribution';
        Lname2 = 'Platoon formation';
        Wsize = 18;
        LWidth = 1;
        MSize  = 8;
        
        % Convergence Time
        
        figure;
        
        
        plot(N_collected,-ObjectiveValue(:,1),'Color',[0.9607,0.1529,0.2196],'linewidth',LWidth); hold on;
        p1 = plot(N_collected,-ObjectiveValue(:,1),'o','Color',[0.9607,0.1529,0.2196],'linewidth',1.5,'markersize',MSize,'displayname',Lname1);
        
        plot(N_collected,-ObjectiveValue(:,2),'Color',[0.0078,0.42,1],'linewidth',LWidth); hold on;
        p2 = plot(N_collected,-ObjectiveValue(:,2),'*','Color',[0.0078,0.42,1],'linewidth',1.5,'markersize',MSize,'displayname',Lname2);
        
        
        switch gammaType
            case 1
                set(gca,'ylim',[-4 0]);
            case 2
                set(gca,'ylim',[-12 0]);
        end
        
        set(gca,'xlim',[8 40]);
        set(gca,'TickLabelInterpreter','latex','fontsize',14);
        
        xl = xlabel('System scale','fontsize',Wsize,'Interpreter','latex','Color','k');
        yl = ylabel('$J(S)$','fontsize',Wsize,'Interpreter','latex','Color','k');
        title(['$k$ = ',num2str(AV_number)],'fontsize',Wsize,'Interpreter','latex','Color','k');
        
        grid on;box on;
        l = legend([p1 p2]);
        %         l.Location = 'SouthWest';
        l.Interpreter = 'latex';
        l.FontSize = Wsize-2;
        l.Box = 'off';
        switch SetupI
            case 1
                l.Position = [0.13,0.2,0.6,0.2];
            case 2
                l.Position = [0.13,0.2,0.6,0.2];
            case 3
                l.Position = [0.15,0.2,0.6,0.2];
            case 4
                l.Position = [0.15,0.2,0.6,0.2];
        end
        
        
        
        set(gca,'xtick',8:8:40);
        
        set(gcf,'Position',[250 150 400 280]);
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        %print(gcf,['Supplementary_figs/Sfigure4_BrakeScenario_TimeComparison_yz'],'-painters','-depsc2','-r400');
        
%         switch SetupI
%             case 1
%                 print(gcf,['Figs/Fig6_Comparison1'],'-painters','-depsc2','-r300');
%             case 2
%                 print(gcf,['Figs/Fig6_Comparison2'],'-painters','-depsc2','-r300');
%             case 3
%                 print(gcf,['Figs/Fig6_Comparison3'],'-painters','-depsc2','-r300');
%             case 4
%                 print(gcf,['Figs/Fig6_Comparison4'],'-painters','-depsc2','-r300');
%         end
        
    end
    
end