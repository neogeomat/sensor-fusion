function [StrideLengths, Thetas, Positions,idx_fig]=Weiberg_StrideLength_Heading_Position(Acc,Gyr,Step_event,StancePhase,ver,idx_fig)

%=========================================================================
% Algorithm que estima la longitud de los pasos usando solo los datos de los acelerómetros. 
% Basado en: [Weinberg2002] Analog devices
% Pasos: 1)Accel magnitude + 2) LowPass + 3)Local Variance + 4) "Begin-of-Stance" event: Detection of sudden variance drop.
%
% INPUT:
%      Acc:      Matriz [nx4], con "n" muestras de los Acelerometros para los ejes X, Y Z y time, en las filas 1, 2 y 3, respectivamente.
%      Gyr:      Matriz [nx4], con "n" muestras de los Gyroscopos para los ejes X, Y, Z y time, en las columnas 1, 2 y 3, respectivamente.
%      Step_event:      Vector conteniendo las k muestras en las que los pasos fueron detectados. La longitud de este vector es el número de pasos 
%      StancePhase:     1 or 0. It is 1 if on stance, 0 otherwise [nx1]%      ver:      0: no ver figuras;  distinto de cero: verlas
%      idx_fig:  entero que indica en que figura hay que hacer la primera representación
% OUTPUT:
%     StrideLengths: Vector indicando la longitud de cada uno de los pasos en metros (la suma de todos lo valores da el avance total)
%     Thetas:        Direction of movement in the horizontal plane after each step. [kx1]
%     Positions:     Position after integration (PDR results)  [kx3]
%     idx_fig:       entero que indica en que figura se debería pintar la proxima vez


K=0.364;  % Cte Weinberg dependiente de cada persona o modo caminar
num_samples_Acc=size(Acc,1);
tiempo_experimento=Acc(end,4)-Acc(1,4);
freq_Acc=ceil(num_samples_Acc/tiempo_experimento);  % samples/s o Hz

num_samples_Gyr=size(Gyr,1);
tiempo_experimento=Gyr(end,4)-Gyr(1,4);
freq_Gyr=ceil(num_samples_Gyr/tiempo_experimento);  % samples/s o Hz

%============================================
% 1) Magnitude
M_Acc=sqrt(Acc(:,1).^2+Acc(:,2).^2+Acc(:,3).^2);

%============================================
% 2) Filtro
[b,a] = butter(4,3/freq_Acc,'low');
M_Acc=filtfilt(b,a,M_Acc);

%============================================
% 3) Formulita AnalogDevices (Weiberg)
sample_ini=1;
StrideLengths=[];
num_Step_events=length(Step_event);
for i=1:num_Step_events % por cada uno de los pasos detectados
    sample_Step_event=Step_event(i);
    Acc_max(i)=max(M_Acc(sample_ini:sample_Step_event));
    Acc_min(i)=min(M_Acc(sample_ini:sample_Step_event));
    Bounce(i)=(Acc_max(i)-Acc_min(i))^(1/4);
    StrideLengths(i)=Bounce(i)*K*2;   
    sample_ini=sample_Step_event;
end

%============================================
% 4) Thetas en cada paso (punta zapato)

% Iniciliazar Matriz Rotación en muestra inicial:
w=1:ceil(10*freq_Acc);  % ventana donde asumo reposo absoluto durante 20 segundos  [muestras]
Acc_mean=[mean(Acc(w,1)),mean(Acc(w,2)),mean(Acc(w,3))];
Roll_ini=atan2(Acc_mean(2),Acc_mean(3))*180/pi;  % Roll en grados
Pitch_ini=-atan2(Acc_mean(1),sqrt(Acc_mean(2)^2+Acc_mean(3)^2))*180/pi;  % Pitch en grados
Yaw_ini=0;
Rot_GS=zeros(3,3,num_samples_Acc);
Rot_z=[cos(Yaw_ini*pi/180) -sin(Yaw_ini*pi/180) 0;  sin(Yaw_ini*pi/180) cos(Yaw_ini*pi/180) 0; 0 0 1];% from S-Sensor to G-Global
Rot_y=[cos(Pitch_ini*pi/180) 0 sin(Pitch_ini*pi/180); 0 1 0; -sin(Pitch_ini*pi/180)   0 cos(Pitch_ini*pi/180)];% from S-Sensor to G-Global
Rot_x=[1 0 0; 0 cos(Roll_ini*pi/180) -sin(Roll_ini*pi/180); 0  sin(Roll_ini*pi/180) cos(Roll_ini*pi/180)];% from S-Sensor to G-Global
Rot_GS(:,:,1)=Rot_z(:,:)*Rot_y(:,:)*Rot_x(:,:);  % from S-Sensor to G-Global BUENA
% Propagar la matriz Rotación a resto de muestras con giroscopo:
for i=1:num_samples_Gyr
    % Actualizar matriz de orientación con valores de los "Gyros"
    SkewGyros=[0 -Gyr(i,3)  Gyr(i,2);...
        Gyr(i,3)  0  -Gyr(i,1);...
        -Gyr(i,2) Gyr(i,1) 0]; % rad/s
    if i>1
        Rot_GS(:,:,i)=Rot_GS(:,:,i-1)*expm(SkewGyros/freq_Gyr); %Funcion "expm" que es un Pade(6,6) BUENA
    end
end
% Por curiosidad veo el angulo de azimut (Yaw)
Roll=squeeze(atan2(Rot_GS(3,2,:),Rot_GS(3,3,:)))*180/pi;
Pitch=squeeze(-asin(Rot_GS(3,1,:)))*180/pi;
Yaw=squeeze(atan2(Rot_GS(2,1,:),Rot_GS(1,1,:)))*180/pi;

% Asumo que Móvil alineado con direccion de movimiento enfrente de usuario
Thetas=zeros(num_Step_events,1);
Step_event_Gyro=Step_event;
for k=2:length(Step_event)
    Step_event_Gyro(k)=floor(Step_event(k)*freq_Gyr/freq_Acc);  % pasar el step_event de muestreo Acc al de Gyr (que puede ser distinto)
end
for k=1:num_Step_events
    Thetas(k)=atan2(Rot_GS(2,1,Step_event_Gyro(k)),Rot_GS(1,1,Step_event_Gyro(k))); % nuevo
end

%============================================
% 5) Positions
Positions=[];
for k=1:num_Step_events
    if (k==1)
        Positions(k,1)=StrideLengths(k)*cos(Thetas(k)); % X
        Positions(k,2)=StrideLengths(k)*sin(Thetas(k)); % Y
    else
        % acumulo con SL y thetas
        Positions(k,1)=Positions(k-1,1)+ StrideLengths(k)*cos(Thetas(k)); % X
        Positions(k,2)=Positions(k-1,2)+ StrideLengths(k)*sin(Thetas(k)); % Y
    end
end
Positions=[0,0; Positions]; % añado posicion 0,0 inicial

%============================================
if ver
     figure(idx_fig);
     plot(StrideLengths,'bo-'); hold on;
     plot(Thetas,'rx-');
     legend({'StrideLengths (m)','Thetas (rad)'}); title('StrideLengths and Thetas');
     xlabel('Steps');
     idx_fig=idx_fig+1;hold off;
    
     % ver Positions y Thetas
     figure(idx_fig);
     plot(Positions(:,1),Positions(:,2),'bo-'); hold on; % marcar trayectoria
     plot(Positions(1,1),Positions(1,2),'bs','MarkerSize',8,'MarkerFaceColor',[0 0 1]); % marcar posicion inicial
     plot(Positions(end,1),Positions(end,2),'bo','MarkerSize',8,'MarkerFaceColor',[0 0 1]); % marcar paso final
     %plot(Thetas(:,1),Positions(:,1),'g');
     ylabel('North (m)'); title('SL+Theta: Positions of trayectories in the global coordinate frame (G)');
     xlabel('East (m)');
     axis equal
     idx_fig=idx_fig+1;hold off;
end