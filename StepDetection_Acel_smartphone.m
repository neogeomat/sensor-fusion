function [Num_steps,StanceBegins_idx,StancePhase,idx_fig]=StepDetection_Acel_smartphone(Acc,ver,idx_fig)

%=========================================================================
%Algorithm that uses only accelerometer data. Based on: [Stirling2003]
% Steps: 1) Acc magnitude + 2) LowPass + 3) Local Variance + 4) "Begin-of-Stance" event: Detection of sudden variance drop.
%
% INPUT:
% Acc: Matrix [nx4], with "n" samples of the Accelerometers for the X, Y , Z , time in columns 1, 2, 3 and 4, respectively.
% see: 0: do not see figures; non-zero: see them
% idx_fig: integer indicating in which figure the first representation must be made
% OUTPUT:
% Num_steps: integer indicating the number of steps detected
% StanceBegins_idx: Events when a step occurs
% StancePhase: 1 or 0. It is 1 if on stance, 0 otherwise
% idx_fig: integer that indicates which figure should be painted next time


% .............. preprocessed ..................
num_samples=size(Acc,1);
tiempo_experimento=Acc(end,4)-Acc(1,4);
freq_Acc=ceil(num_samples/tiempo_experimento);  % samples/s o Hz
Acc_mag=sqrt(Acc(:,1).^2+Acc(:,2).^2+Acc(:,3).^2);
orden_filtro=4;
[b,a] = butter(orden_filtro,2/(freq_Acc/2),'low'); % IIR low order filter design of order 4 (cut-off freq 2 Hz)
[Acc_mag_filt,zf]=filter(b,a,Acc_mag,[[9.73783786570634;-22.8430473945528;18.4412646506037;-5.04055548881593]]);

% .............Detectar pasos ..................
umbral_Acc=0.4; % threshold of 0.4 m / s ^ 2
umbral_Acc_descarte=2.0; % threshold greater than 1.5 m / s ^ 2 indicating excessive wiggle
gravity=9.8;
Acc_filt_binary=zeros(1,length(Acc));
Acc_filt_detrend=zeros(1,length(Acc));
for ii=2:length(Acc)
    gravity=0.999*gravity+0.001*Acc_mag(ii);  % I calculate gravity experimentally (since in S3 it gives me 9.7 instead of what is expected to be 9.8)
    Acc_filt_detrend(ii)=Acc_mag_filt(ii)-gravity;
    if Acc_filt_detrend(ii)>umbral_Acc && Acc_filt_detrend(ii)<umbral_Acc_descarte
        Acc_filt_binary(ii)=1; % fases de subida del cuerpo (inicio paso), pongo "1"
    else
        if Acc_filt_detrend(ii)<-umbral_Acc
            if Acc_filt_binary(ii-1)==1   % if after an "1" there is no intermediate value to generate the "0" then I will force it
                Acc_filt_binary(ii)=0;
            else  % normal case: I have already set "1", then "0", and now "-1"
                Acc_filt_binary(ii)=-1; % phases of lowering the body (end step)
            end
        else
            Acc_filt_binary(ii)=0; % if it is between the upper and lower threshold => I mark it with "0"
        end
    end
    
end
stepcount=0;
StanceBegins_idx=[];
StepDect=zeros(1,length(Acc));
steps=zeros(1,length(Acc))*NaN;
window=ceil(0.4*freq_Acc); % muestras en ventana para considerar 0.4 segundos 
			   % samples in window to consider 0.4 seconds
for ii=(window+2):length(Acc)
    % detect steps if:
    if ( Acc_filt_binary(ii)==-1  && Acc_filt_binary(ii-1)==0  && (sum(Acc_filt_binary(ii-window:ii-2))>1 ))
        StepDect(ii)=1;
        stepcount=stepcount+1; % contador de pasos
        StanceBegins_idx(stepcount)=ii;
    end
    if StepDect(ii)
        steps(ii)=0;
    else
        steps(ii)=NaN;
    end
end

% All support samples
StancePhase=zeros(num_samples,1);
for ii=StanceBegins_idx
   StancePhase(ii:ii+9,1)=ones(10,1);  % I assume that the support phase is the next 10 samples at the beginning of the support phase (StanceBegins_idx)
end

% Calculation parameters:
Num_steps=length(StanceBegins_idx); % number of steps counted

figure(idx_fig); idx_fig=idx_fig+1;
vect_smaples=1:num_samples;
plot(vect_smaples,Acc_mag,'r-'); hold on;  title('"SL+theta PDR": Acclerometer processing for Step detection'); ylabel('Accleration'); xlabel('time (seconds)');
plot(vect_smaples,Acc_mag_filt,'b-');
plot(vect_smaples,Acc_filt_detrend,'c-');
plot(vect_smaples,Acc_filt_binary,'gx-');
plot(vect_smaples,steps,'ro','MarkerSize',8); hold off;
legend({'|Acc|','lowpass(|Acc|)','detrend(lowpass(|Acc|))','Binary','detected Steps'});

