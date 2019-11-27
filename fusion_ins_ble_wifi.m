clc; clear; clf; close all; disp('PRACTICE 3: PDR with hand-held real smartphone IMU (Samsung S4) ');
format long g
format compact
%
%........................................................................................
%% 1) Load real data with smart-phone IMU 

% Read log_file
disp('1) Load real data with smart-phone IMU and inspect');% fflush(stdout);
disp('Reading Logfile...');% fflush(stdout);

savefile = 'fusion_ble_ins_wifi';
if isfile([savefile '.mat'])
    load(savefile);
else[Acc,Gyr,Ble4,Gnss,Wifi,Posi,~,~]=ReadLogFile('.\log_files\library_campaign_6_with_german.txt','smartphone',1);
    
end
disp('Logfile Read...');%fflush(stdout);
disp('-> TO DO: Inspect IMU signals and bias (press enter to continue)');%fflush(stdout);
% pause;

% read real coordinates of campaign
campaign6 = csvread('wifi_datasets\campaign06.csv',1,0);
campaign6 = dataset({campaign6 'X','Y','Number','Floor'});

if(strcmp(class(Posi),'double'))
   Posi = dataset({Posi 'Timestamp','Counter','X','Y','floorID','BuildingID'});
end

for row = Posi.Counter
    Posi.X = campaign6.X(Posi.Counter == campaign6.Number(row));
    Posi.Y = campaign6.Y(Posi.Counter == campaign6.Number(row));
    Posi.floorID = campaign6.Floor(Posi.Counter == campaign6.Number(row));
end

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

%% 3) get wifi positions from knn
% load Wifi training data
addpath('wifi_datasets');
dataTrainWifi = loadTrainData();
dataTrainMerged = mergedata(dataTrainWifi,6);
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
knnValue = 3;    % Number of neighbors
predictionKnn = kNNEstimation(dataTrainWifi.rss, dataTestWifi, dataTrainWifi.coords, 3);
% Wifiplots(dataTrainWifi.rss, dataTestWifi, dataTrainWifi.coords, knnValue);

%% 4) Get BLE positions using WC
% read beacons positions and major,minor values
BleBeacons = csvread('ble\BLEbeacons.csv',1,0);
BleBeacons = dataset({BleBeacons 'X','Y','Major','Minor'});
BleBeacons = sortrows(BleBeacons,4); % sort by minor value
% convert Ble4 from log file to dataTestBle
addpath('ble');
Ble4 = Ble4(string(Ble4.UUID) == 'A9C04048-A71E-42B5-B569-13B5AC77B618',:); % filter readings that are from beacons deployed by us, discard other BLE sources
window = 6; %in seconds
% calculate mean if same ble device appears more than once in the window
dataTestBleCount = zeros(ceil(max(Ble4.TimeStamp)/window),length(BleBeacons.Minor));
dataTestBleSum = zeros(ceil(max(Ble4.TimeStamp)/window),length(BleBeacons.Minor));
for w = 1:length(Ble4.RSS)
    col = Ble4.MinorID(w);
    row = ceil(Ble4.TimeStamp(w)/window);
    
    dataTestBleSum(row,col) = dataTestBleSum(row,col) + Ble4.RSS(w);
    dataTestBleCount(row,col) = dataTestBleCount(row,col) + 1;
end
dataTestBle = dataTestBleSum ./ dataTestBleCount; % mean if same ble device in a window
dataTestBle(dataTestBle == 0) = NaN;
bcCoords = double(BleBeacons(:,[1 2]));
predictionWC = wCEstimation(bcCoords,dataTestBle,3);
predictionDelaunay = delaunayEstimation_with_weight(bcCoords,dataTestBle,3);

%% 5) get timestamps from position sources (BLE, INS, WIFI)
% initialize timestamp collectors
AppTimeStamp = []; Source = [];
if (exist('Acc','var'))
    AppTimeStamp = [AppTimeStamp; Acc.AppTimestamp(Step_events)];
    Source = [Source; ones(length(Acc.AppTimestamp(Step_events)),1)]; % 1 = Acce
end
% if (exist('Posi','var'))
%     AppTimeStamp = [AppTimeStamp; Posi.Timestamp];
%     Source = [Source; repmat(16,length(Posi.Timestamp),1)]; % 16 = Posi
% end
if (exist('Ble4','var'))
    AppTimeStamp = [AppTimeStamp; (window:window:window*size(dataTestBle,1))']; % because Ble4 positions are given at fixed window intervals
    Source = [Source; repmat(2,size(dataTestBle,1),1)]; % 2 = Ble4
end
% if (exist('Wifi','var'))
%     AppTimeStamp = [AppTimeStamp; Ble4.TimeStamp];
%     Source = [Source; repmat(23,length(Ble4.TimeStamp),1)]; % 23 = Wifi
% end

% triggers is made up of AppTimeStamp and Source
% AppTimeStamp: TimeStamp from GetSensorData logfile, only for position
% providers
% Source: Sensor which provides recording at that time
triggers = dataset([AppTimeStamp],[Source],'VarNames',{'AppTimestamp','Source'});
triggers_sort = sortrows(triggers,1); % sort by TimeStamp
triggers_sort_posi = find(double(triggers_sort(:,2))==16); % 16 = Posi
triggers_sort_Ble4 = find(double(triggers_sort(:,2))==2); % 2 = Ble4
%% 3) Fusion of BLE, INS and Wifi
p_hat = zeros(2,Num_steps); % X,V
A = [1 0;...
    0 1]; % transformation matrix
P = eye(size(A)); % Prcess Covarience
C = eye(size(A));% Observation Model
Z = [Posi.X-Posi.X(1) Posi.Y-Posi.Y(1)]';% Measurement
R = [cov(StrideLengths) 0 ;
    0 cov(Thetas)]; % Measurement covarience
Q = zeros(size(A)); % Process Covarience
I = eye(size(A));

index_acce = 1; index_posi = 1; index_Ble4 = 1; index_Wifi = 1;
% p_hat(:,1) = [StrideLengths(1)*cos(Thetas(1)),StrideLengths(1)*sin(Thetas(1))];
p_hat(:,1) = [Posi.X(1);Posi.Y(1)];
% p_hat(:,1) = [predictionWC(1,1);predictionWC(1,2)];
clf
axis equal
for k = 1:length(triggers_sort) % first trigger is posi but there is no estimate to update at this time
    trigger = triggers_sort(k,2);
    switch trigger{1,1} % trigger is cell array from dataset
        case 1 % 1 = 'Acce'
            % prediction
            index_acce = index_acce + 1;
            if index_acce == 1
%                 p_hat(:,1) = [StrideLengths(1)*cos(Thetas(1));StrideLengths(1)*sin(Thetas(1))]; 
            else
                U = StrideLengths(index_acce-1) * [cos(Thetas(index_acce-1)); sin(Thetas(index_acce-1))];
                p_hat(:,index_acce) = A * p_hat(:,index_acce - 1) ...
                    + U;
                P = A * P * A' + Q;
            end
            
            plot(p_hat(1,1:index_acce-1),p_hat(2,1:index_acce-1),'r-o');
            hold on
            
        case 2 % 2 = 'Ble4'
            % update
            
            K         = P  * C' / (C * P * C' + R);
            P         = (I - K * C) * P;
            p_hat(:,index_acce) = p_hat(:,index_acce) ...
                + K * (predictionWC(index_Ble4,:)' - C * p_hat(:,index_acce));
            
            plot(predictionWC(1:index_Ble4,1),predictionWC(1:index_Ble4,2),'g-o');
            plot([predictionWC(index_Ble4,1);p_hat(1,index_acce)'],...
                [predictionWC(index_Ble4,2);p_hat(2,index_acce)'],'b:o');
            hold on
            index_Ble4 = index_Ble4 + 1;
        otherwise
            continue
    end
    pause(0.1)
end

% clf
% plot(Positions(:,1),Positions(:,2))
plot(Positions(:,1) + Posi.X(1),Positions(:,2) + Posi.Y(1),'k-')
% hold on
% plot(p_hat(1,:)-predictionWC(1,1),p_hat(2,:)-predictionWC(1,2))
% hold on 
% % plot(Posi.X,Posi.Y)
% plot(Posi.X - Posi.X(1),Posi.Y - Posi.Y(1))
% hold on
% plot(predictionWC(:,1) - Posi.X(1),predictionWC(:,2) - Posi.Y(1))
% % plot(predictionWC(:,1),predictionWC(:,2))
% legend({'IMU','fusion IMU BLE','Campaign6','BLE WC Positions'})

save(savefile);