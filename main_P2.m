% -----------------------------------------
%         PDR Tutorial: PRACTICE 2 (main_P2.m)
%       Lopsi Group. CAR-CSIC/UPM
%                  2017
%------------------------------------------

clc; clear all; close all; disp('PRACTICE 2: PDR with FOOT-MOUNTED real XSENS-MEMS IMU');
%
%        PRACTICE 2: PDR with FOOT-MOUNTED real XSENS-MEMS IMU 
% 1) Load real data with foot-mounted IMU (logfile_3loops_1lateralbackwards.txt)
%       -One square 3 times (76: 22+23+31 steps, forward walk, but last 3 sides lateral/backwards walk)
% 2) Apply INS PDR algorithm and analyse drifts in position
% 3) Apply INS-ZUPT and analyse processing: 
%       -Detection of steps and stance 
%       -Correction of Velocities to zero (ZUPT)
%       -Walking direction

%........................................................................................
% 1) Load real data with foot-mounted IMU (logfile_3loops_1lateralbackwards.txt)
%       -One square 3 times (76: 22+23+31 steps, forward walk, but last 3 sides lateral/backwards walk)

% Read log_file
disp('1) Load real data with foot-mounted test and inspect'); fflush(stdout);
disp('Reading Logfile...'); fflush(stdout);
% load IMU read data: Acc,Gyr de Xsens (3 loops)
[~,~,Acc,Gyr]=ReadLogFile('.\log_files\logfile_3loops_1lateralbackwards.txt','Xsens',1);  % 76 steps (2 loops + 1 loop lateral/backwards)
%[~,~,Acc,Gyr]=ReadLogFile('.\log_files\logfile_3loops_forward.txt','Xsens',1);  % 66 steps (3 loops)
disp('Logfile Read...');fflush(stdout);
disp('-> TO DO: Inspect IMU signals and bias (press enter to continue)');fflush(stdout);
pause;

%...........................................................................................
% 2) Apply INS PDR algorithm and analyse drifts in position (remove bias)
disp(sprintf('\n2) Apply INS PDR algorithm and analyse drifts in position (remove bias)'));
% Remove bias Gyro
samples=5000;  % asumo 50 segundos parado (y fs=100 Hz)
bias_Gyr=[mean(Gyr(1:samples,1)), mean(Gyr(1:samples,2)), mean(Gyr(1:samples,3))];
Gyr_unbiased=Gyr;  %[nx4]
Gyr_unbiased(:,1:3)=[Gyr(:,1)-bias_Gyr(1), Gyr(:,2)-bias_Gyr(2), Gyr(:,3)-bias_Gyr(3)];

% Apply INS to obtain Pos,Vel y Att:
disp('Applying INS...');fflush(stdout);
[Pos_G_rec,Vel_G_rec,Att_G_rec]=INS(Acc,Gyr);
%[Pos_G_rec,Vel_G_rec,Att_G_rec]=INS(Acc,Gyr_unbiased);
disp('INS ended. Showing results...');fflush(stdout);
idx_fig=10;
visualizar_INS_results(Pos_G_rec,Vel_G_rec,Att_G_rec,idx_fig);
disp('-> TO DO: Check uncorrect INS integration/ remove bias (press enter to continue)');fflush(stdout);
pause;

%............................................................................................
% 3) Apply INS-ZUPT and analyse processing: 
%        -Detection of steps and stance 
%        -Correction of Velocities to zero (ZUPT)
%        -Walking direction

disp(sprintf('\n3) Apply INS-ZUPT and analyse processing'));fflush(stdout);
disp('Applying INS-ZUPT...');fflush(stdout);
%-----Step detection------
idx_fig=20;
[Num_steps,Step_events,StancePhase,idx_fig]=StepDetection_Acel(Acc,1,idx_fig);
%-----INS-ZUPT-----------
%[StrideLengths, Thetas, Positions,idx_fig]=ZUPT_StrideLength_Heading_Position(Acc,Gyr,Step_events,StancePhase,1,idx_fig);
[StrideLengths, Thetas, Positions,idx_fig]=ZUPT_StrideLength_Heading_Position(Acc,Gyr_unbiased,Step_events,StancePhase,1,idx_fig);
   
disp('INS-ZUPT ended. Showing results...');fflush(stdout);
disp('-> TO DO: Check correct integration INS-ZUPT');fflush(stdout);



