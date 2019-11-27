clc; clear; clf; close all; disp('PRACTICE 3: PDR with hand-held real smartphone IMU (Samsung S4) ');
format long g
format compact
%
%........................................................................................
%% 1) Load real data with smart-phone IMU 

% Read log_file
disp('1) Load real data with smart-phone IMU and inspect');% fflush(stdout);
disp('Reading Logfile...');% fflush(stdout);
savefile = 'fusion_ins_posi';
if isfile([savefile '.mat'])
    load(savefile);
else
    [Acc,Gyr,Ble4,Gnss,Wifi,Posi,~,~]=ReadLogFile('.\log_files\library_campaign_6_with_german.txt','smartphone',1);
end
campaign6 = csvread('wifi_datasets\campaign06.csv',1,0);
campaign6 = dataset({campaign6 'X','Y','Number','Floor'});
Posi = dataset({Posi 'Timestamp','Counter','X','Y','floorID','BuildingID'});
for row = Posi.Counter
    Posi.X = campaign6.X(Posi.Counter == campaign6.Number(row));
    Posi.Y = campaign6.Y(Posi.Counter == campaign6.Number(row));
    Posi.floorID = campaign6.Floor(Posi.Counter == campaign6.Number(row));
end
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
samples=3000;  % value for library_campaign_6_with_german.txt
% samples = length(Gyr.ST);
% I assume 50 seconds stopped (and fs=100 Hz)
% bias_Gyr=[mean(Gyr(1:samples,1)), mean(Gyr(1:samples,2)), mean(Gyrs(1:samples,3))];
bias_Gyr=[mean(double(Gyr.Gyr_x(1:samples))), ...
    mean(double(Gyr.Gyr_y(1:samples))), ...
    mean(double(Gyr.Gyr_z(1:samples)))];
Gyr_unbiased=Gyr;  %[nx4]
% Gyr_unbiased(:,1:3)=[Gyr(:,1)-bias_Gyr(1), Gyr(:,2)-bias_Gyr(2), Gyr(:,3)-bias_Gyr(3)];
% Gyr_unbiased=[double(Gyr.Gyr_x)-bias_Gyr(1), double(Gyr.Gyr_y)-bias_Gyr(2), double(Gyr.Gyr_z)-bias_Gyr(3),double(Gyr.ST)];
Gyr_unbiased.Gyr_x = double(Gyr.Gyr_x)-bias_Gyr(1);
Gyr_unbiased.Gyr_y = double(Gyr.Gyr_y)-bias_Gyr(2);
Gyr_unbiased.Gyr_z = double(Gyr.Gyr_z)-bias_Gyr(3);
Gyr_unbiased.ST    = double(Gyr.AppTimestamp);
% Apply INS to obtain Pos,Vel y Att:
disp('Apply SL+theta PDR...');%fflush(stdout);
%-----Step detection------
idx_fig=200;
%[Num_steps,Step_events,StancePhase,idx_fig]=StepDetection_Acel(Acc,1,idx_fig);
[Num_steps,Step_events,StancePhase,idx_fig] = StepDetection_Acel_smartphone(...
    double(Acc),0,idx_fig);
%-----SL-theta-----------
%[StrideLengths, Thetas, Positions,idx_fig]=Weiberg_StrideLength_Heading_Position(Acc,Gyr,Step_events,StancePhase,1,idx_fig);
[StrideLengths, Thetas, Positions,idx_fig] = Weiberg_StrideLength_Heading_Position(...
    double(Acc),double(Gyr_unbiased),Step_events,StancePhase,1,idx_fig);

disp(['-> TO DO: -Check bias remove effect',...
    '            -Step detection & Stride Length estimation',...
    '            -Position estimation while walking lateral/backwards']);%fflush(stdout);
%% 3) Get wifi positions using k-mean
% load Wifi training data
addpath('wifi_datasets');
dataTrainWifi = loadTrainData();
dataTrainWifiMerged = mergedata(dataTrainWifi,6);
% deal with not seen AP
dataTrainWifi.rss(dataTrainWifi.rss==100) = -110;

% get list of MACs in Wifi Training Data
file_wifi_macs = 'wifi_datasets\tst01-mac-head.csv';
wifiMacUnique = getUniqueWifiMac(file_wifi_macs);

% convert wifi from log file to test data
wifiTimeUnique = unique(Wifi.AppTimestamp);
dataTestWifi = zeros(length(wifiTimeUnique),length(wifiMacUnique));
for w = 1:length(Wifi)
    row = find(wifiTimeUnique == Wifi.AppTimestamp(w));
    col = find(wifiMacUnique == Wifi.MAC(w));
    dataTestWifi(row,col) = Wifi.RSS(w);
end
dataTestWifi(dataTestWifi==0) = -110;
% kNN method estimation
knnValue = 9;    % Number of neighbors
predictionKnn = kNNEstimation(dataTrainWifiMerged.rss, dataTestWifi, dataTrainWifiMerged.coords, 3);
Wifiplots(dataTrainWifiMerged.rss, dataTestWifi, dataTrainWifiMerged.coords, knnValue);
%% 4) Get BLE positions using WC
% read beacons positions and major,minor values
BleBeacons = csvread('ble\BLEbeacons.csv',1,0);
BleBeacons = dataset({BleBeacons 'X','Y','Major','Minor'});
BleBeacons = sortrows(BleBeacons,4); % sort by minor value
% convert Ble4 from log file to dataTestBle
addpath('ble');
Ble4 = Ble4(string(Ble4.UUID) == 'A9C04048-A71E-42B5-B569-13B5AC77B618',:); % filter readings that are from beacons deployed by us, discard other BLE sources
window = 6; %in seconds
dataTestBleCount = zeros(ceil(max(Ble4.timestamp)/window),length(BleBeacons.Minor));
dataTestBleSum = zeros(ceil(max(Ble4.timestamp)/window),length(BleBeacons.Minor));
for w = 1:length(Ble4.RSS)
    col = Ble4.MinorID(w);
    row = ceil(Ble4.timestamp(w)/window);
    
    dataTestBleSum(row,col) = dataTestBleSum(row,col) + Ble4.RSS(w);
    dataTestBleCount(row,col) = dataTestBleCount(row,col) + 1;
end
dataTestBle = dataTestBleSum ./ dataTestBleCount;
dataTestBle(dataTestBle == 0) = NaN;
bcCoords = double(BleBeacons(:,[1 2]));
predictionWC = wCEstimation(bcCoords,dataTestBle,3);
predictionDelaunay = delaunayEstimation_with_weight(...
    bcCoords,dataTestBle,3);
%% 5) Plot different results 
clf;
% campaign6.X = campaign6.X - min(campaign6.X);
% campaign6.Y = campaign6.Y - min(campaign6.Y);

plot(campaign6.X,campaign6.Y,'x-');
axis equal
hold on

positionsINS = Positions;
plot(positionsINS(:,1) + campaign6.X(1),...
    positionsINS(:,2) + campaign6.Y(1),...
    'g-o')

% adjusted_predictonKnn = predictionKnn - min(predictionKnn);
adjusted_predictonKnn = predictionKnn;
plot(adjusted_predictonKnn(:,1),adjusted_predictonKnn(:,2),'b--o')

BBX = BleBeacons.X - min(BleBeacons.X);
BBY = BleBeacons.Y - min(BleBeacons.Y);
% plot(BBX,BBY,'dg');
plot(BleBeacons.X,BleBeacons.Y,'dc');

adjusted_predictonWC = predictionWC(~isnan(predictionWC(:,1)),:);
plot(adjusted_predictonWC(:,1),adjusted_predictonWC(:,2),'r--o')
%% test Ble in different window sizes
Bleplots(bcCoords,Ble4,campaign6)
%% test wifi in different k values
Wifiplots(dataTrainWifiMerged.rss, dataTestWifi,...
    dataTrainWifiMerged.coords, knnValue);
%%
adjusted_predictonDelaunay = predictionDelaunay %- min(predictionDelaunay);
plot(adjusted_predictonDelaunay(:,1),adjusted_predictonDelaunay(:,2),'k--o')
legend({'campaign6','INS Positions','Wifi Positions','Ble beacons','BLE Positions WC','BLE Positions Delaunay'})

