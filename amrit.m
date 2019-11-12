clc; close all; disp('PRACTICE 3: PDR with hand-held real smartphone IMU (Samsung S4) ');
%
%........................................................................................
% 1) Load real data with smart-phone IMU 

% Read log_file
disp('1) Load real data with smart-phone IMU and inspect');% fflush(stdout);
disp('Reading Logfile...');% fflush(stdout);
% load IMU read data: Acc,Gyr de Xsens (3 loops)
%[~,~,Acc,Gyr]=ReadLogFile('.\log_files\logfile_3loops_1lateralbackwards.txt','Xsens',1); %ON FOOT %(2 loops + 1 loop lateral/backwards)
% [Acc,Gyr,~,~]=ReadLogFile('.\log_files\Around living room carpet logfile_2019_10_06_11_31_51.txt','smartphone',1); %ON HAND %(2 loops + 1 loop lateral/backwards)
[Acc,Gyr,Ble4,Gnss,Wifi,~,~]=ReadLogFile('.\log_files\library_last_day_collection.txt','smartphone',1);

disp('Logfile Read...');%fflush(stdout);
disp('-> TO DO: Inspect IMU signals and bias (press enter to continue)');%fflush(stdout);
% pause;

%...........................................................................................
% 2) Apply SL+theta PDR algorithm and analyse results
%        -Check bias remove effect 
%        -Step detection & Stride Length estimation 
%        -Position estimation while walking lateral/backwards
fprintf('\n2) Apply SL+theta PDR algorithm and analyse rsults\n');%fflush(stdout);
% Remove bias Gyro
samples=95;  % asumo 50 segundos parado (y fs=100 Hz)
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
% [Num_steps,Step_events,StancePhase,idx_fig]=StepDetection_Acel_smartphone(double(Acc),0,idx_fig);
%-----SL-theta-----------
%[StrideLengths, Thetas, Positions,idx_fig]=Weiberg_StrideLength_Heading_Position(Acc,Gyr,Step_events,StancePhase,1,idx_fig);
% [StrideLengths, Thetas, Positions,idx_fig]=Weiberg_StrideLength_Heading_Position(double(Acc),double(Gyr_unbiased),Step_events,StancePhase,1,idx_fig);

disp(['-> TO DO: -Check bias remove effect',...
    '            -Step detection & Stride Length estimation',...
    '            -Position estimation while walking lateral/backwards']);%fflush(stdout);

%............................................................................................
% 3) Get wifi positions using k-mean
% load training data

% convert wifi from log file to test_data
uniqueWifiMac = unique(Wifi.MAC);
uniqueWifiTime = unique(Wifi.timestamp);
test_data = zeros(length(uniqueWifiTime),length(uniqueWifiMac));
for w = 1:length(Wifi)
    midx = find(uniqueWifiMac == Wifi.MAC(w));
    tidx = find(uniqueWifiTime == Wifi.timestamp(w));
    
    test_data(tidx,midx) = Wifi.RSS(w);
end
% wifi_k_means('test_data','train_data')




