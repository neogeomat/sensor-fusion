% -----------------------------------------
%        PDR Tutorial: PRACTICE 1 (main_P1.m)
%       Lopsi Group. CAR-CSIC/UPM
%                  2017
%------------------------------------------
clc; clear all; close all; disp('PRACTICE 1: Effects of noise on INS');
%                PRACTICE 1: Effects of noise on INS
% 1) Load ideal noise-free data (ideal_footIMU_rectangulo.txt) and inspect
% 2) Apply INS algorithm (INS.m) to ideal IMU signals and check correct integration
% 3) Add noise (rand or bias) to IMU signals and inspect drift in velocity and position generated

%........................................................................................
% 1) Load ideal noise-free data (ideal_footIMU_rectangulo.txt) and inspect

% Read log_file
disp('1) Load ideal noise-free data (ideal_footIMU_rectangulo.txt) and inspect'); fflush(stdout);
disp('Reading Logfile...');fflush(stdout);
[~,~,Acc,Gyr]=ReadLogFile('.\log_files\ideal_footIMU_rectangulo.txt','Xsens');  % load IMU simulated data: Acc,Gyr,Imux 
load('.\log_files\ideal_footIMU_rectangulo_GT.mat'); % load Ground_truth data: 'Pos_G','Vel_G','Att_G','Rot_GS','Stance','StepDectSample','gravedad','fs'
disp('Logfile Read...');fflush(stdout);
disp('-> TO DO: Inspect IMU signals  (press enter to continue)');fflush(stdout);
pause;

%...........................................................................................
% 2) Apply INS algorithm (INS.m) to ideal IMU signals and check correct integration
disp(sprintf('\n2) Apply INS algorithm (INS.m) to ideal IMU signals and check correct integration'));fflush(stdout);
% Apply INS to obtain Pos,Vel y Att:
disp('Applying INS...');fflush(stdout);
[Pos_G_rec,Vel_G_rec,Att_G_rec]=INS(Acc,Gyr);
disp('INS ended. Showing results...');fflush(stdout);
visualizar_INS_results(Pos_G_rec,Vel_G_rec,Att_G_rec,Pos_G,Vel_G,Att_G,Rot_GS,Stance,StepDectSample,10);
disp('-> TO DO: Check correct integration (press enter to continue)');fflush(stdout);
pause;

%............................................................................................
% 3) Add noise (rand or bias) to IMU signals and inspect drift
disp(sprintf('\n3) Add noise (rand or bias) to IMU signals and inspect drift'));fflush(stdout)
Amplitud=0.0;  % m/s^2
Bias=0.0;       % m/s^2
samples=length(Acc);
Acc(:,1:3)=Acc(:,1:3)+Amplitud*randn(samples,3);
Acc(:,3)=Acc(:,3)+Bias*ones(samples,1);

Amplitud=0.0;  % rad/s
Bias=0.001;       % rad/s
samples=length(Gyr);
Gyr(:,1:3)=Gyr(:,1:3)+Amplitud*randn(samples,3);
Gyr(:,3)=Gyr(:,3)+Bias*ones(samples,1);

% Apply INS to obtain Pos,Vel y Att:
disp('Applying INS with noise...');fflush(stdout);
[Pos_G_rec,Vel_G_rec,Att_G_rec]=INS(Acc,Gyr);
disp('INS with noise ended. Showing results...');fflush(stdout);
visualizar_INS_results(Pos_G_rec,Vel_G_rec,Att_G_rec,Pos_G,Vel_G,Att_G,Rot_GS,Stance,StepDectSample,20);
disp('-> TO DO: Check correct integration (press enter to continue)');fflush(stdout);
