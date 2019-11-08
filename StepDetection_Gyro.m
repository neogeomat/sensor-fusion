function [Num_steps,StanceSample,StancePhase,idx_fig]=StepDetection_Gyro(Gyr,ver,idx_fig)

%=========================================================================
% Algorithm 1: Step Detetion using Gyros
% Rate gyro magnitude + Threshold + Medium + "Begin-of-Stance" event
% Based on: [Happy 2009]
%
% INPUT:
% Gyr: Matrix [nx3], with "n" gyro samples, for the X, Y and Z axes, in rows 1, 2 and 3, respectively.
% see: 0: do not see figures; non-zero: see them
% idx_fig: integer indicating in which figure the first representation must be made
% OUTPUT:
% Num_steps: integer indicating the number of steps detected
% idx_fig: integer that indicates which figure should be painted next time


num_samples=size(Gyr,1);

% initialize variables
M_Gyr3=zeros(1,num_samples);

% Step1: rate gyro magnitude
M_Gyr=sqrt(Gyr(:,1).^2+Gyr(:,2).^2+Gyr(:,3).^2);

% Step 2: Thresholding
Th_M_Gyr=1; % 1 rad/s
M_Gyr2=int8(M_Gyr>Th_M_Gyr)*Th_M_Gyr;

% Step 3: medium filter (window 0.20 sec; 20 samples)
half_size=10;
M_Gyr3(1:half_size)=M_Gyr2(1:half_size);
for i=half_size+1:length(M_Gyr2)-half_size
    valores=M_Gyr2(i-half_size:i+half_size-1);
    M_Gyr3(i)=median(valores);
end
M_Gyr3(length(M_Gyr2)-half_size+1:length(M_Gyr2))=...
    M_Gyr2(length(M_Gyr2)-half_size+1:length(M_Gyr2));
M_Gyr4=M_Gyr3;

%% Step 3: dilatation filter + erosion (0.20 sec window; 20 samples)
% half_size = 6;
%% dilatation
% M_Gyr3 (1: half_size) = M_Gyr2 (1: half_size);
% for i = half_size + 1: length (M_Gyr2) -half_size
% values = M_Gyr2 (i-half_size: i + half_size-1);
% if max (values) == Th_M_Gyr
% M_Gyr3 (i) = Th_M_Gyr;
% else
% M_Gyr3 (i) = M_Gyr2 (i);
% end
% end
% M_Gyr3 (length (M_Gyr2) -half_size + 1: length (M_Gyr2)) = M_Gyr2 (length (M_Gyr2) -half_size + 1: length (M_Gyr2));
%% erosion
% M_Gyr4 (1: half_size) = M_Gyr3 (1: half_size);
% for i = half_size + 1: length (M_Gyr2) -half_size
% values = M_Gyr3 (i-half_size: i + half_size-1);
% if min (values) == 0
% M_Gyr4 (i) = 0;
% else
% M_Gyr4 (i) = M_Gyr3 (i);
% end
% end
% M_Gyr4 (length (M_Gyr3) -half_size + 1: length (M_Gyr3)) = M_Gyr3 (length (M_Gyr3) -half_size + 1: length (M_Gyr3));

% Step 4: Begin of stance event

M_Gyr5(1)=0;
M_Gyr5(2:length(M_Gyr4))=M_Gyr4(2:length(M_Gyr4))<M_Gyr4(1:length(M_Gyr4)-1);
StanceSample=find(M_Gyr5);

% All support samples
StancePhase=zeros(num_samples,1);
for i=StanceSample
   StancePhase(i:i+9,1)=ones(10,1);  % I assume that the support phase is the next 10 samples at the beginning of the support phase (StanceBegins_idx)
end


% Calculation parameters:
Num_steps=length(StanceSample); % number of steps counted

if ver
    % see gyro magnitudes
    figure(idx_fig);
    plot(Gyr(:,1),'r'); hold on;
    plot(Gyr(:,2),'g');
    plot(Gyr(:,3),'b');
    plot(M_Gyr,'k','LineWidth',2);
    ylabel('rad/s'); title('Angular rate');
    xlabel('samples');
    legend('w_x','w_y','w_z','Magnitude');
    idx_fig=idx_fig+1;hold off;
    
    % see intermediate stages of step detection
    figure(idx_fig);
    plot(M_Gyr,'k','LineWidth',1); hold on;
    plot(M_Gyr2,'g','LineWidth',2);
    plot(M_Gyr4,'b','LineWidth',2);
    plot(StanceSample,zeros(1,Num_steps),'ro','MarkerSize',10,'MarkerFaceColor',[1 0 0]);
    ylabel('rad/s'); title('Step detection with angular rate thresholding');
    xlabel('samples');
    legend('Magnitude','Threshold','Median filter','Stance-Begins');
    idx_fig=idx_fig+1; hold off;
end