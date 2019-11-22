close all, clc, clear;
[~,~,~,~,Wifi,~,~]=ReadLogFile('library_campaign_6_with_german.txt','smartphone',1);
%% 3) Get wifi positions using k-mean
%% load Wifi training data
data = loadTrainData();
dataTrainWifi.rss = data(:,1:180);
dataTrainWifi.coords = data(:,[182 181 183]); % in the csv files, X and Y are swapped: X = 182, Y = 181,Floor = 183
dataTrainMerged = mergedata(dataTrainWifi,6);
% deal with not seen AP
dataTrainWifi.rss(dataTrainWifi.rss==100) = -110;

%% get list of MACs in Wifi Training Data
fid = fopen('tst01-mac-head.csv');
hdr = fgetl(fid);
fclose(fid);
macs = regexp(hdr,',','split');
mac_dec = zeros(1,length(macs)-5); % last 5 are X,Y,Floor,Timestamp,sampleid 
% for ap = 1:length(macs) - 5
for mac_idx = 1:(length(macs) -5)
    MAC_str = macs(mac_idx);
    MAC_dec_array=sscanf(MAC_str{1,1},'%x:%x:%x:%x:%x:%x');
    mac_dec(mac_idx) = MAC_dec_array(1)*256^5 ...
        + MAC_dec_array(2)*256^4 ...
        + MAC_dec_array(3)*256^3 ...
        + MAC_dec_array(4)*256^2 ...
        + MAC_dec_array(5)*256 ...
        + MAC_dec_array(6);
end

%% convert wifi from log file to test data
% uniqueWifiMac = unique(Wifi.MAC);
wifiMacUnique = mac_dec;
wifiTimeUnique = unique(Wifi.timestamp);
dataTestWifi = zeros(length(wifiTimeUnique),length(wifiMacUnique));
for w = 1:length(Wifi)
    row = find(wifiTimeUnique == Wifi.timestamp(w)); % find row
    column = find(wifiMacUnique == Wifi.MAC(w)); % find column
    
    dataTestWifi(row,column) = Wifi.RSS(w);
end
% deal with not seen AP
dataTestWifi(dataTestWifi==0) = -110;
%% kNN method estimation
knnValue = 9;    % Number of neighbors
predictionKnn = kNNEstimation(dataTrainWifi.rss, dataTestWifi, dataTrainWifi.coords, knnValue);
Wifiplots(dataTrainWifi.rss, dataTestWifi, dataTrainWifi.coords, knnValue);
%% Campaign 6
campaign6 = csvread('campaign06.csv',1,0);
campaign6 = dataset({campaign6 'X','Y','Number','Floor'});
% plot(campaign6.X,campaign6.Y,'x-');