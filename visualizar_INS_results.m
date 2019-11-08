function visualizar_INS_results(Pos_G_rec,Vel_G_rec,Att_G_rec,Pos_G,Vel_G,Att_G,Rot_GS,Stance,StepDectSample,idx_fig)

if nargin<=4
    hay_GT=false;
    subplots=3;
    if nargin==4, idx_fig=Pos_G; end
    if nargin==3, idx_fig=10; end
else
    hay_GT=true;
    subplots=4;
end

ver=1;   
num_muestras=length(Pos_G_rec);
if ver
    figure(idx_fig);hold off;
    plot3(Pos_G_rec(:,1),Pos_G_rec(:,2),Pos_G_rec(:,3),'r-'); %trayectoria reconstruida
    grid on; title('Position 3D:X-Y-Z en G (m)');    %axis equal
    hold on;
    if hay_GT==true
        plot3(Pos_G(:,1),Pos_G(:,2),Pos_G(:,3),'b-'); ylabel('PosyG'); zlabel('PoszG'); xlabel('PosxG'); % trayectoria ideal
        plot3(Pos_G(StepDectSample,1),Pos_G(StepDectSample,2),Pos_G(StepDectSample,3),'ro','MarkerSize',6,'MarkerFaceColor',[0 0 1]);
        plot3(Pos_G_rec(StepDectSample,1),Pos_G_rec(StepDectSample,2),Pos_G_rec(StepDectSample,3),'ro','MarkerSize',6,'MarkerFaceColor',[1 0 0]);
        legend('simulated trajectory','stances','INS re-generated trajectory','INS stances');
    end
    idx_fig=idx_fig+1;hold off;
       
    figure(idx_fig); hold off;
    v_samples=[1:num_muestras];
    subplot(subplots,1,1);  ylabel('PosxG'); hold on; plot(v_samples,Pos_G_rec(:,1),'r-.'); grid on;title('Position G (m)'); 
    subplot(subplots,1,2);  ylabel('PosyG');hold on; plot(v_samples,Pos_G_rec(:,2),'r-.'); grid on; 
    subplot(subplots,1,3);  ylabel('PoszG');hold on; plot(v_samples,Pos_G_rec(:,3),'r-.'); grid on; 
    if hay_GT==true
        subplot(subplots,1,1); plot(Pos_G(:,1)); legend('INS re-generated','simulated');
        subplot(subplots,1,2); plot(Pos_G(:,2));
        subplot(subplots,1,3); plot(Pos_G(:,3));
        subplot(subplots,1,4); plot(Stance,'g-'); ylabel('Stance & Step'); axis([get(gca,'XLim') -0.1 1.1]);
        hold on; plot(StepDectSample,zeros(1,length(StepDectSample)),'ro','MarkerSize',6,'MarkerFaceColor',[0 0 1]);
    end
    xlabel('samples');
    idx_fig=idx_fig+1;hold off;
    
    figure(idx_fig);hold off;
    subplot(subplots,1,1);  hold on; plot(v_samples,Vel_G_rec(:,1),'r-.'); ylabel('VelxG'); grid on;title('Velocity G (m/s)');
    subplot(subplots,1,2);  hold on; plot(v_samples,Vel_G_rec(:,2),'r-.'); ylabel('VelyG');grid on;
    subplot(subplots,1,3);  hold on; plot(v_samples,Vel_G_rec(:,3),'r-.'); ylabel('VelzG');grid on;
    if hay_GT==true
        subplot(subplots,1,1); plot(Vel_G(:,1)); legend('INS re-generated','simulated');
        subplot(subplots,1,2); plot(Vel_G(:,2));
        subplot(subplots,1,3); plot(Vel_G(:,3));
       subplot(subplots,1,4); plot(Stance,'g-'); ylabel('Stance & Step'); axis([get(gca,'XLim') -0.1 1.1]);
       hold on; plot(StepDectSample,zeros(1,length(StepDectSample)),'ro','MarkerSize',6,'MarkerFaceColor',[0 0 1]);
    end
    xlabel('samples');
    idx_fig=idx_fig+1;hold off;
   
    figure(idx_fig);hold off;
    subplot(subplots,1,1);  hold on; plot(v_samples,Att_G_rec(:,1),'r-.'); ylabel('Roll'); grid on;title('Attitude G (degrees)'); 
    subplot(subplots,1,2);  hold on; plot(v_samples,Att_G_rec(:,2),'r-.'); ylabel('Pitch');grid on;
    subplot(subplots,1,3);  hold on; plot(v_samples,Att_G_rec(:,3),'r-.'); ylabel('Yaw');grid on;
    if hay_GT==true
        subplot(subplots,1,1); plot(Att_G(:,1)); legend('INS re-generated','simulated');
        subplot(subplots,1,2); plot(Att_G(:,2));
        subplot(subplots,1,3); plot(Att_G(:,3)); 
       subplot(subplots,1,4); plot(Stance,'g-'); ylabel('Stance & Step'); axis([get(gca,'XLim') -0.1 1.1]);
       hold on; plot(StepDectSample,zeros(1,length(StepDectSample)),'ro','MarkerSize',6,'MarkerFaceColor',[0 0 1]);
    end
    idx_fig=idx_fig+1;hold off;
    
end