function Wifiplots(rss, dataTestWifi, coords, knnValue)
    windows = 1:knnValue; %in seconds
    figure('Name','Wifi knn plots')
    for window = windows
        subplot(3,3,window)
        
        predictionKnn = kNNEstimation(rss, dataTestWifi, coords, window);
        plot(predictionKnn(:,1),predictionKnn(:,2),'k--o')
        hold on
%         adjusted_predictonDelaunay = predictionDelaunay;
%         plot(adjusted_predictonDelaunay(:,1),adjusted_predictonDelaunay(:,2),'k--o')
        hold off
        title(['k = ',num2str(window),' seconds']) 
        axis equal        
    end
end