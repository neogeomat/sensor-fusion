clc; clear; clf; close all; disp('PRACTICE 3: PDR with hand-held real smartphone IMU (Samsung S4) ');
format long g
format compact
%
%........................................................................................
%% 1) Load real data with smart-phone IMU 

% Read log_file
disp('1) Load real data with smart-phone IMU and inspect');% fflush(stdout);
disp('Reading Logfile...');% fflush(stdout);
% load IMU read data: Acc,Gyr de Xsens (3 loops)
%[~,~,Acc,Gyr]=ReadLogFile('.\log_files\logfile_3loops_1lateralbackwards.txt','Xsens',1); %ON FOOT %(2 loops + 1 loop lateral/backwards)
% [Acc,Gyr,~,~]=ReadLogFile('.\log_files\Around living room carpet logfile_2019_10_06_11_31_51.txt','smartphone',1); %ON HAND %(2 loops + 1 loop lateral/backwards)
% [Acc,Gyr,Ble4,Gnss,Wifi,~,~]=ReadLogFile('.\log_files\library_last_day_collection.txt','smartphone',1);
[Acc,Gyr,Ble4,Gnss,Wifi,~,~]=ReadLogFile('.\log_files\library_campaign_6_with_german.txt','smartphone',1);
disp('Logfile Read...');%fflush(stdout);
disp('-> TO DO: Inspect IMU signals and bias (press enter to continue)');%fflush(stdout);
% pause;

%...........................................................................................
%% 2) Apply SL+theta PDR algorithm and analyse results
%        -Check bias remove effect 
%        -Step detection & Stride Length estimation 
%        -Position estimation while walking lateral/backwards
fprintf('\n2) Apply SL+theta PDR algorithm and analyse rsults\n');%fflush(stdout);
% Remove bias Gyro
% samples=95;  % value for library_last_day_collection.txt
samples=5000;  % value for library_campaign_6_with_german.txt
% samples = length(Acc);
% I assume 50 seconds stopped (and fs=100 Hz)
% bias_Gyr=[mean(Gyr(1:samples,1)), mean(Gyr(1:samples,2)), mean(Gyrs(1:samples,3))];
bias_Gyr=[mean(double(Gyr.Gyr_x(1:samples))), mean(double(Gyr.Gyr_y(1:samples))), mean(double(Gyr.Gyr_z(1:samples)))];
Gyr_unbiased=Gyr;  %[nx4]
% Gyr_unbiased(:,1:3)=[Gyr(:,1)-bias_Gyr(1), Gyr(:,2)-bias_Gyr(2), Gyr(:,3)-bias_Gyr(3)];
% Gyr_unbiased=[double(Gyr.Gyr_x)-bias_Gyr(1), double(Gyr.Gyr_y)-bias_Gyr(2), double(Gyr.Gyr_z)-bias_Gyr(3),double(Gyr.ST)];
Gyr_unbiased.Gyr_x = double(Gyr.Gyr_x)-bias_Gyr(1);
Gyr_unbiased.Gyr_y = double(Gyr.Gyr_y)-bias_Gyr(2);
Gyr_unbiased.Gyr_z = double(Gyr.Gyr_z)-bias_Gyr(3);
Gyr_unbiased.ST    = double(Gyr.ST);
% Apply INS to obtain Pos,Vel y Att:
disp('Apply SL+theta PDR...');%fflush(stdout);
%-----Step detection------
idx_fig=200;
%[Num_steps,Step_events,StancePhase,idx_fig]=StepDetection_Acel(Acc,1,idx_fig);
[Num_steps,Step_events,StancePhase,idx_fig]=StepDetection_Acel_smartphone(double(Acc),0,idx_fig);
%-----SL-theta-----------
%[StrideLengths, Thetas, Positions,idx_fig]=Weiberg_StrideLength_Heading_Position(Acc,Gyr,Step_events,StancePhase,1,idx_fig);
[StrideLengths, Thetas, Positions,idx_fig]=Weiberg_StrideLength_Heading_Position(double(Acc),double(Gyr_unbiased),Step_events,StancePhase,1,idx_fig);

disp(['-> TO DO: -Check bias remove effect',...
    '            -Step detection & Stride Length estimation',...
    '            -Position estimation while walking lateral/backwards']);%fflush(stdout);

%............................................................................................
%% 3) Get wifi positions using k-mean
%% load Wifi training data
addpath('wifi_datasets');
% data1 = csvread('wifi_datasets\tst01-mac-head.csv',1,0);
% data2 = csvread('wifi_datasets\tst02-mac-head.csv',1,0);
% data3 = csvread('wifi_datasets\tst03-mac-head.csv',1,0);
% data4 = csvread('wifi_datasets\tst04-mac-head.csv',1,0);
% data5 = csvread('wifi_datasets\tst05-mac-head.csv',1,0);
% data = [data1;data2;data3;data4;data5];
data = loadTrainData();
dataTrainWifi.rss = data(:,1:180);
dataTrainWifi.coords = data(:,181:183);
dataTrainMerged = mergedata(dataTrainWifi,6);

%% get list of MACs in Wifi Training Data
fid = fopen('wifi_datasets\tst01-mac-head.csv');
hdr = fgetl(fid);
fclose(fid);
macs = regexp(hdr,',','split');
mac_dec = zeros(1,length(macs)-5);
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
    midx = find(wifiMacUnique == Wifi.MAC(w));
    tidx = find(wifiTimeUnique == Wifi.timestamp(w));
    
    dataTestWifi(tidx,midx) = Wifi.RSS(w);
end

%% kNN method estimation
knnValue = 9;    % Number of neighbors
predictionKnn = kNNEstimation(dataTrainMerged.rss, dataTestWifi, dataTrainMerged.coords, knnValue);

%% 4) Get BLE positions using WC
BleBeacons = csvread('ble\BLEbeacons.csv',1,0);
BleBeacons = dataset({BleBeacons 'X','Y','Major','Minor'});
BleBeacons = sortrows(BleBeacons,4);
%% convert wifi from log file to dataTestBle
addpath('ble');
Ble4 = Ble4(string(Ble4.UUID) == 'A9C04048-A71E-42B5-B569-13B5AC77B618',:); % filter readings that are from beacons deployed by us, discard other BLE sources
window = 10; %in seconds
dataTestBle = zeros(ceil(max(Ble4.timestamp)/window),length(BleBeacons.Minor));
for w = 1:length(Ble4.RSS)
    midx = Ble4.MinorID(w);
    tidx = ceil(Ble4.timestamp(w)/window);
    
    dataTestBle(tidx,midx) = Ble4.RSS(w);
end
dataTestBle(dataTestBle == 0) = NaN;
bcCoords = double(BleBeacons(:,[1 2]));
predictionWC = wCEstimation(bcCoords,dataTestBle,3);
dataTestBle_del.rss = dataTestBle;
predictionDelaunay = delaunayEstimation_with_weight(bcCoords,dataTestBle_del,3);

%% test Ble in different window sizes
Bleplots(bcCoords,Ble4)
%% 5) Plot different results 
clf;
campaign6 = csvread('wifi_datasets\campaign06.csv',1,0);
campaign6 = dataset({campaign6 'X','Y','Number','Floor'});
campaign6.X = campaign6.X - min(campaign6.X);
campaign6.Y = campaign6.Y - min(campaign6.Y);

plot(campaign6.X,campaign6.Y,'x-');
axis equal
hold on

positionsINS = Positions;
% plot(positionsINS(:,1) + campaign6.X(1),positionsINS(:,2) + campaign6.Y(1),'g-o')

adjusted_predictonKnn = predictionKnn - min(predictionKnn);
% plot(adjusted_predictonKnn(:,1),adjusted_predictonKnn(:,2),'b--o')

BBX = BleBeacons.X - min(BleBeacons.X);
BBY = BleBeacons.Y - min(BleBeacons.Y);
plot(BBX,BBY,'dg');

adjusted_predictonWC = predictionWC - min(predictionWC);
plot(adjusted_predictonWC(:,1),adjusted_predictonWC(:,2),'r--o')

adjusted_predictonDelaunay = predictionDelaunay - min(predictionDelaunay);
plot(adjusted_predictonDelaunay(:,1),adjusted_predictonDelaunay(:,2),'k--o')
legend({'campaign6','INS Positions','Wifi Positions','Ble beacons','BLE Positions WC','BLE Positions Delaunay'})

