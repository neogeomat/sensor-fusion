%%
function [dataTrainWifi] =  loadTrainData()
data1 = csvread('wifi_datasets\tst01-mac-head.csv',1,0);
data2 = csvread('wifi_datasets\tst02-mac-head.csv',1,0);
data3 = csvread('wifi_datasets\tst03-mac-head.csv',1,0);
data4 = csvread('wifi_datasets\tst04-mac-head.csv',1,0);
data5 = csvread('wifi_datasets\tst05-mac-head.csv',1,0);
data = [data1;data2;data3;data4;data5];
dataTrainWifi.rss = data(:,1:180);
dataTrainWifi.coords = data(:,[182 181 183]);
% keyboard
end

