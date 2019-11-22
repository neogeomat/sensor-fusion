%%
function [data] =  loadTrainData()
data1 = csvread('tst01-mac-head.csv',1,0);
data2 = csvread('tst02-mac-head.csv',1,0);
data3 = csvread('tst03-mac-head.csv',1,0);
data4 = csvread('tst04-mac-head.csv',1,0);
data5 = csvread('tst05-mac-head.csv',1,0);
data = [data1;data2;data3;data4;data5];
end

