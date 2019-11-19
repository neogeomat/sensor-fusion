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
[Acc,Gyr,Ble4,Gnss,Wifi,Posi,~,~]=ReadLogFile('.\log_files\library_campaign_6_with_german.txt','smartphone',1);
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

%% get timestamps from position sources
% initialize timestamp collectors
AppTimeStamp = []; Source = [];
if (exist('Acc','var'))
    AppTimeStamp = [AppTimeStamp; Acc.AppTimestamp(Step_events)];
    Source = [Source; repmat({'Acce'},length(Acc.AppTimestamp(Step_events)),1)];
end
if (exist('Posi','var'))
    AppTimeStamp = [AppTimeStamp; Posi.Timestamp];
    Source = [Source; repmat({'Posi'},length(Posi.Timestamp),1)];
end
% triggers is made up of AppTimeStamp and Source
% AppTimeStamp: TimeStamp from GetSensorData logfile, only for position
% providers
% Source: Sensor which provides recording at that time
triggers = dataset([AppTimeStamp],[Source],'VarNames',{'AppTimestamp','Source'});
triggers_sort = sortrows(triggers,1);
%% 3) Fusion of INS and Posi
% Number of observations
Num_steps = length(Step_events);

p_hat = zeros(2, Num_steps); % X,V
% xhat(:,1) = [Posi.X(1),Posi.Y(1)];
p_hat(:,1) = [0,0];
% p_hat(:,1) = [StrideLengths(1)*cos(Thetas(1)),StrideLengths(1)*sin(Thetas(1))];
for k = 1:length(triggers_sort)
    trigger = triggers_sort(k,2);
    switch trigger{1,1}
        case 'Acce'
            % prediction
            if ~exist('index_p')
                index_p = 1;
                p_hat(:,1) = [0,0];
                
                continue;
            end
            index_p = index_p + 1;
            A = [1 0;...
                0 1];
            B = [StrideLengths(index_p) 0; ...
                0, StrideLengths(index_p)];
            U = StrideLengths(index_p) * [cos(Thetas(index_p)); sin(Thetas(index_p))];
            p_hat(:,index_p) = A * p_hat(:,index_p - 1) + U;
        otherwise
            % update
            continue
    end
end
% p_hat = [[0;0] p_hat];

% Positions(k,1)=Positions(k-1,1)+ StrideLengths(k)*cos(Thetas(k)); % X
% Positions(k,2)=Positions(k-1,2)+ StrideLengths(k)*sin(Thetas(k)); % Y
clf
plot(p_hat(1,:),p_hat(2,:))
hold on 
plot(Positions(:,1),Positions(:,2))
plot(Posi.X - Posi.X(1),Posi.Y - Posi.Y(1))
legend({'p_[hat]','IMU','Campaign6'})

