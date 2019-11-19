function Bleplots(bcCoords,Ble4,truePosi)
    windows = 4:7; %in seconds
    figure('Name','Bleplots')
    for window = windows
        
        dataTestBle = zeros(ceil(max(Ble4.timestamp)/window),length(bcCoords));
        for w = 1:length(Ble4.RSS)
            midx = Ble4.MinorID(w);
            tidx = ceil(Ble4.timestamp(w)/window);

            dataTestBle(tidx,midx) = Ble4.RSS(w);
        end
        dataTestBle(dataTestBle == 0) = NaN;
        predictionWC = wCEstimation(bcCoords,dataTestBle,3);
        dataTestBle_del.rss = dataTestBle;
        predictionDelaunay = delaunayEstimation_with_weight(bcCoords,dataTestBle_del,3);
        
        subplot(2,2,window - windows(1) + 1)
        
        adjusted_predictonWC = predictionWC(~isnan(predictionWC(:,1)),:);
        plot(adjusted_predictonWC(:,1),adjusted_predictonWC(:,2),'b--*')
        hold on
        adjusted_predictonDelaunay = predictionDelaunay(~isnan(predictionDelaunay(:,1)),:);
        plot(adjusted_predictonDelaunay(:,1),adjusted_predictonDelaunay(:,2),'k-.+');
        hold on
        plot(truePosi.X,truePosi.Y,'x:g');
        hold off
        
        title(['window size ',num2str(window),' seconds']) 
        
        axis equal
    end
legend({'WC estimates','Delaynay Estimates','True Posi'})
end