% delaunayESTIMATION Estimate a position for each query element using delauney method.
%
%	[PREDICTION,NEAREST,KS] = delaunayESTIMATION(BCCOORS, QUERY, K) creates a position
%	prediction in PREDICTION for each row in QUERY. A prediction is
%	computed as the weighted centroid of the positions in BCCOORS
%	associated to the K beacons whose intensity values were the strongest.
%	The amount of beacons whose position were used for estimation is is
%	returned in KS.
%
%   See Also KNNESTIMATION.
%
%   Copyright � 2018 Universitat Jaume I (UJI)

% Permission is hereby granted, free of charge, to any person obtaining a copy of
% this software and associated documentation files (the "Software"), to deal in
% the Software without restriction, including without limitation the rights to
% use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
% of the Software, and to permit persons to whom the Software is furnished to do
% so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

function [predictionWW,nearest,ks] = delaunayEstimation_with_weight(bcCoors, data, k)
    savefig = 0;
    query = data.rss;
    [queryRows,nBeacons] = size(query);
    
    if (~exist('k','var'))
        k = nBeacons;
    end

    prediction = zeros(queryRows, 2);
    predictionWW = zeros(queryRows, 2);
    predictionWC = zeros(queryRows, 2);
    nearest = zeros(queryRows, 25); % one SN 24 RSS APS
    
    ks = zeros(queryRows, 1);

    ws = weights(query);
    
    % testing cutoff of lower signal
%     p = ws;
%     p(isnan(p)) = 0;
%     p =sort(p,2,'descend');
%     d = zeros(116,23);
%     for i = 1:23
%         d(:,i) = p(:,i) - p(:,i+1);
%     end
% %     cut = mean(mean(d));
% %     cut = prctile(prctile(d,50),50);
%     cut = 0.0000003;
%     cutd = d - cut
%     cutd(cutd <0) = 0
    % --------
    for i = (1:queryRows)
        if(savefig)
            clf        
            % Plot Beacons
            % ------------
            hold on
            bid = 1:length(bcCoors);
            plot(bcCoors(:,1),bcCoors(:,2),'hg');
            text(bcCoors(:,1),bcCoors(:,2),string(bid'),'VerticalAlignment','bottom','HorizontalAlignment','left');
        end
        % ------------
        found = ~isnan(ws(i,:));
        [qWs,I] = sort(ws(i,found),'descend');
        qBpos = bcCoors(found,:);
        qBpos = qBpos(I,:);
        
        % WC method
        nk = min(k,sum(found));
        ks(i) = nk;

        predictionWC(i,:) = (((qBpos((1:nk),:)')*qWs(1:nk)')./sum(qWs(1:nk)))';        
        %
        f = find(found == 1);
        beaconsByNearestOrder = f(I);
        if(length(qBpos(:,1))>=3) % if no of closest points is less than 3, triangle cannot be formed, so the method doesn't work. Assign NaN in such case
           for t = (3:length(qBpos(:,1)))     
               if(~predictionWW(i,:)) % position might have been found in previous iterations of t
                    DT = delaunayTriangulation(qBpos(1:t,:));
                    if(DT.ConnectivityList)
                        [e,~]=size(DT.ConnectivityList); 
                        WCTs = [];
                        % find no of Delaunay triangles formed 
                        for o = (1:e)
                            angles = triangle_angles(DT.Points(DT.ConnectivityList(o,:),:),'d');
                            if(angles > 30 & angles < 120)
                                WCTs = [WCTs, o];
                                if(savefig)
                                hold on
                                line(DT.Points(DT.ConnectivityList(o,:)',1),DT.Points(DT.ConnectivityList(o,:)',2))
                                end
                            end
                        end
                        if(WCTs)
                            prediction(i,:) = mean(incenter(DT,WCTs'),1);
                            % find pos in each triangle
                            pos = [];
                            for o = WCTs
                                pos = [pos; (((qBpos(DT.ConnectivityList(o,:),:)')*qWs(DT.ConnectivityList(o,:))')./sum(qWs(DT.ConnectivityList(o,:))))'];
                            end
                            predictionWW(i,:) = mean(pos,1);
                            if(savefig)
                            hold on
%                             line([data.coords(i,1);qBpos(:,1)],[data.coords(i,2);qBpos(:,2)],'LineSTyle',':');
                            end
                            break
                        end
                    end
               end
           end
        end
        if(~prediction(i,:))
            prediction(i,:) = NaN; %output NaN like in prediction instead of zero
        end
        if(~predictionWW(i,:))
            predictionWW(i,:) = NaN; %output NaN like in prediction instead of zero
        end
        
        nearest(i,:) = [i,beaconsByNearestOrder,zeros(1,length(nearest(i,:))-1-length(f(I)))];
        
        %-------
        if(savefig)
        hold on
        plot(prediction(i,1),prediction(i,2),'*r')
        text(prediction(i,1),prediction(i,2),string(i),'VerticalAlignment','bottom','HorizontalAlignment','left')
        hold on
        plot(predictionWW(i,1),predictionWW(i,2),'+g')
        text(predictionWW(i,1),predictionWW(i,2),string(i),'VerticalAlignment','bottom','HorizontalAlignment','left')
        hold on
        plot(data.coords(i,1),data.coords(i,2),'ob')
        text(data.coords(i,1),data.coords(i,2),string(i),'VerticalAlignment','bottom','HorizontalAlignment','left')
        hold on
        plot(predictionWC(i,1),predictionWC(i,2),'dk')
        text(predictionWC(i,1),predictionWC(i,2),string(i),'VerticalAlignment','bottom','HorizontalAlignment','left')
        saveas(gcf,['amrit/triplots/triplot_lib_merged_s6_' num2str(i) '.png']);
        end
    end
end

function [w] = weights(rssi)
    w = 10.^(rssi/10);
end