% WCESTIMATION Estimate a position for each query element using weighted
% centroid.
%
%	[PREDICTION,KS] = WCESTIMATION(BCCOORS, QUERY, K) creates a position
%	prediction in PREDICTION for each row in QUERY. A prediction is
%	computed as the weighted centroid of the positions in BCCOORS
%	associated to the K beacons whose intensity values were the strongest.
%	The amount of beacons whose position were used for estimation is is
%	returned in KS.
%
%   See Also KNNESTIMATION.
%
%   Copyright © 2018 Universitat Jaume I (UJI)

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

function [prediction,ks] = wCEstimation(bcCoors, query, k)

    [queryRows,nBeacons] = size(query);
    
    if (~exist('k','var'))
        k = nBeacons;
    end

    prediction = zeros(queryRows, 2);
    ks = zeros(queryRows, 1);

    ws = weights(query);

    for i = (1:queryRows)
        found = ~isnan(ws(i,:));

        [qWs,I] = sort(ws(i,found),'descend');
        qBpos = bcCoors(found,:);qBpos = qBpos(I,:);

        nk = min(k,sum(found));
        ks(i) = nk;

        prediction(i,:) = (((qBpos((1:nk),:)')*qWs(1:nk)')./sum(qWs(1:nk)))';
    end
end

function [w] = weights(rssi)
    w = 10.^(rssi/10);
end

