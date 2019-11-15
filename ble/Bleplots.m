function Bleplots(bcCoords,Ble4)
    windows = 1:10; %in seconds
    figure('Name','Bleplots')
    for window = windows
        
        dataTestBle = zeros(ceil(max(Ble4.timestamp)/window),length(bcCoords));
        for w = 1:length(Ble4.RSS)
            midx = Ble4.MinorID(w);
            tidx = ceil(Ble4.timestamp(w)/window);

            dataTestBle(tidx,midx) = Ble4.RSS(w);
        end
        dataTestBle(dataTestBle == 0) = NaN;
%         bcCoords = double(BleBeacons(:,[1 2]));
        predictionWC = wCEstimation(bcCoords,dataTestBle,3);
        dataTestBle_del.rss = dataTestBle;
%         predictionDelaunay = delaunayEstimation_with_weight(bcCoords,dataTestBle_del,3);
        subplot(2,5,window)
        axis equal
        adjusted_predictonWC = predictionWC;
        plot(adjusted_predictonWC(:,1),adjusted_predictonWC(:,2),'r--o')
        hold on
%         adjusted_predictonDelaunay = predictionDelaunay;
%         plot(adjusted_predictonDelaunay(:,1),adjusted_predictonDelaunay(:,2),'k--o')
        hold off
        title(['window size ',num2str(window),' seconds']) 
        
    end
end