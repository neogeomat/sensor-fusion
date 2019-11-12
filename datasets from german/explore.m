clear; 
clc; 
close all;

data1 = csvread('tst01.csv',1,0);
data2 = csvread('tst02.csv',1,0);
data3 = csvread('tst03.csv',1,0);
data4 = csvread('tst04.csv',1,0);
data = [data1;data2;data3;data4];

latCol = 181;
lonCol = 182;
floorCol = 183;
idCol = 185;
APCol = 4;

% idx = floor(rem(data(:,idCol),10^5)/100);
idx = floor(data(:,idCol)/100);
[C, ~, ic] = unique(idx);

lats = accumarray(ic,data(:,latCol),[],@mean);
lons = accumarray(ic,data(:,lonCol),[],@mean);
flrs = accumarray(ic,data(:,floorCol),[],@mean);

rss = data(:,APCol); rss(rss==100) = nan;
rsss = accumarray(ic,rss,[],@myMean);

scatter3(lats, lons, flrs*10, [], rsss, 'filled');
colormap(jet);
caxis([-110,-40]);
colorbar;
axis image;

function result = myMean(myData)
    result = mean(myData,'omitnan');
end