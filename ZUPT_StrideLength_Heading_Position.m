function [StrideLengths, Thetas, Positions, idx_fig]=ZUPT_StrideLength_Heading_Position(Acc,Gyr,Step_event,StancePhase,ver,idx_fig)

%=========================================================================
% Algorithm que estima la longitud de los pasos usando ZUPT con los acelerómetros, y la matriz de rotacion. 
% Basado en: [Feliz2009]
% Pasos: 1)Accel a global coordinates + 2) Integration to linear velocity + 3) Linear Velo correction + 4) ....
%
% INPUT:
%      Acc:      Matriz [nx3], con "n" muestras de los Acelerometros para los ejes X, Y y Z, en las columnas 1, 2 y 3, respectivamente.
%      Gyr:      Matriz [nx3], con "n" muestras de los Gyroscopos para los ejes X, Y y Z, en las columnas 1, 2 y 3, respectivamente.
%      Step_event:      Vector conteniendo las k muestras en las que los pasos fueron detectados. La longitud de este vector es el número de pasos 
%      StancePhase:     1 or 0. It is 1 if on stance, 0 otherwise [nx1]
%      ver:      0: no ver figuras;  distinto de cero: verlas
%      idx_fig:  entero que indica en que figura hay que hacer la primera representación
% OUTPUT:
%     StrideLengths: Vector [kx1] indicando la longitud en metros de cada uno de los k pasos (la suma de todos lo valores da el avance total)
%     Thetas:        Direction of movement in the horizontal plane after each step. [kx1]
%     Positions:     Position after integration (PDR results)  [kx3]
%     idx_fig:       entero que indica en que figura se debería pintar la proxima vez

fs=100;  % frecuencia muestreo 100 Hz (0.01 s)
num_samples=size(Acc,1);

% Inicializar variables
 Acc_G=zeros(num_samples,3);
 Velo_G=zeros(num_samples,3);
 Velo_G_corr=zeros(num_samples,3);
 
% Still condition  (ver cuando estoy quieto totalmente):
still_count=0;
still=zeros(1,num_samples);
th_tiempo_still=ceil(1.5*fs);  % 1.5 segundos  [samples]
for i=1:num_samples
   mag_Gyr=sqrt(Gyr(i,1)^2+Gyr(i,2)^2+Gyr(i,3)^2); 
   if mag_Gyr<0.2  % rad/s
       still_count=still_count+1;  % cuento muestras (samples con giro sin moverse)
   else
       still_count=0;
   end
   if still_count>th_tiempo_still  % si más de 1.5 segundo parado => estoy en still (calma absoluta)
       still(i)=1;
   else
       still(i)=0;
   end
end

%=========================================
% 1) Linear Accelerations from SENSOR (S) to GLOBAL (G) coordinates (uso quaterniones definiendo orientacion absoluta de IMU)
w=1:ceil(20*fs);  % ventana donde asumo reposo absoluto durante 20 segundos  [muestras]
gravedad=mean(sqrt(Acc(w,1).^2+Acc(w,2).^2+Acc(w,3).^2));  % m/s^2

% Iniciliazar Matriz Rotación:
Acc_mean=[mean(Acc(w,1)),mean(Acc(w,2)),mean(Acc(w,3))];
Roll_ini=atan2(Acc_mean(2),Acc_mean(3))*180/pi;  % Roll en grados
Pitch_ini=-atan2(Acc_mean(1),sqrt(Acc_mean(2)^2+Acc_mean(3)^2))*180/pi;  % Pitch en grados
Yaw_ini=0;
Rot_GS=zeros(3,3,num_samples);
Rot_z=[cos(Yaw_ini*pi/180) -sin(Yaw_ini*pi/180) 0;  sin(Yaw_ini*pi/180) cos(Yaw_ini*pi/180) 0; 0 0 1];% from S-Sensor to G-Global
Rot_y=[cos(Pitch_ini*pi/180) 0 sin(Pitch_ini*pi/180); 0 1 0; -sin(Pitch_ini*pi/180)   0 cos(Pitch_ini*pi/180)];% from S-Sensor to G-Global
Rot_x=[1 0 0; 0 cos(Roll_ini*pi/180) -sin(Roll_ini*pi/180); 0  sin(Roll_ini*pi/180) cos(Roll_ini*pi/180)];% from S-Sensor to G-Global
Rot_GS(:,:,1)=Rot_z(:,:)*Rot_y(:,:)*Rot_x(:,:);  % from S-Sensor to G-Global BUENA
for i=1:num_samples
    % Actualizar matriz de orientación con valores de los "Gyros"
    SkewGyros=[0 -Gyr(i,3)  Gyr(i,2);...
        Gyr(i,3)  0  -Gyr(i,1);...
        -Gyr(i,2) Gyr(i,1) 0]; % rad/s
    if i>1
        Rot_GS(:,:,i)=Rot_GS(:,:,i-1)*expm(SkewGyros/fs); %Funcion "expm" que es un Pade(6,6) BUENA
    end
end
% Por curiosidad veo el angulo de azimut (Yaw)
Roll=squeeze(atan2(Rot_GS(3,2,:),Rot_GS(3,3,:)))*180/pi;
Pitch=squeeze(-asin(Rot_GS(3,1,:)))*180/pi;
Yaw=squeeze(atan2(Rot_GS(2,1,:),Rot_GS(1,1,:)))*180/pi;

% Tranformar aceleraciones lineales
for i=1:num_samples
   Acc_G(i,1:3)=(Rot_GS(:,:,i)*Acc(i,1:3)')';  % Rot_GS pasa del sistema S al G (Rot_GS' pasa del sistema G al S)
end
Acc_G(:,3)=Acc_G(:,3)-gravedad;

%============================================
% 2) Integration to linear velocities
Velo_G(1,:)=[0 0 0];
for i=2:num_samples
  Velo_G(i,:)=Velo_G(i-1,:)+Acc_G(i,:)/fs;
end

%============================================
% 3) Linear Velo correction 
% Mean velocity error at stance
num_steps=size(Step_event,2);
disp(['Number of steps: ',num2str(num_steps)]);
Error_Velo_G=zeros(num_steps,3);
Step_event_ext=[1 Step_event num_samples]; % añado el ultimo y el primero como puntos de apoyo
for k=1:num_steps+2
    Error_Velo_G(k,:)=Velo_G(Step_event_ext(k),:);
end
% Corregir velo  mediante iterpolación (de todas las muestras)
for k=2:num_steps+2
    for i=Step_event_ext(k-1):Step_event_ext(k)-1
        samples_in_step=Step_event_ext(k)-Step_event_ext(k-1);
        if still(i)==1  % si calma absoluta pongo velo a cero
            Velo_G_corr(i,:)=[0 0 0];
        else
            Velo_G_corr(i,:)=Velo_G(i,:)-(Error_Velo_G(k,:)*(i-Step_event_ext(k-1)) + Error_Velo_G(k-1,:)*(Step_event_ext(k)-i))/samples_in_step;
        end
    end
end

%============================================
% 4) Stride length estimation 
% Velo integration to obtain linear displacement
StrideLengths=[];
for k=1:num_steps
    Aux_G=[0 0 0]; % desplazamiento en x y z
    for i=Step_event_ext(k):Step_event_ext(k+1)-1
        Aux_G=Aux_G+Velo_G_corr(i,:)/fs;
    end
    StrideLengths(k)=sqrt(Aux_G(1)^2+Aux_G(2)^2); % longitud paso en plano X-Y 
    Pos_delta(k,:)=[Aux_G(1) -Aux_G(2) Aux_G(3)]; % X-Y-Z position incremets (North-West-Up)  (pongo -1 para pasar de oeste a este)
end

%============================================
% 5) Thetas
Thetas=[];
for k=1:num_steps % Thetas (angulo detectato a partir de los incrementos de posicion)
    Thetas(k)=atan2(Pos_delta(k,2),Pos_delta(k,1)); % orientacion respecto el Norte en el plano
end


%============================================
% 6) Positions
% Acumulo con Pos_deltas (full_ZUPT)
Positions=[];
for k=1:num_steps
    if (k==1)
        Positions(k,1)=Pos_delta(k,1); % X
        Positions(k,2)=Pos_delta(k,2); % Y
        Positions(k,3)=Pos_delta(k,3); % Y
    else
        Positions(k,1)=Positions(k-1,1)+ Pos_delta(k,1); % X-Norte
        Positions(k,2)=Positions(k-1,2)+ Pos_delta(k,2); % Y-Este
        Positions(k,3)=Positions(k-1,3)+ Pos_delta(k,3); % Z-Up or down
    end
end
Positions=[0,0,0; Positions]; % añado posicion 0,0,0 inicial



%============================================
if ver
     % Ver el angulo del sensor
     figure(idx_fig);
     plot([1:num_samples],Roll,'r'); hold on;
     plot([1:num_samples],Pitch,'g'); 
     plot([1:num_samples],Yaw,'b'); 
     plot(Step_event,zeros(1,size(Step_event,2)),'ro','MarkerSize',6,'MarkerFaceColor',[1 0 0]);
     xlabel('samples'); ylabel('Degrees');
     legend('Roll','Pitch','Yaw');
     idx_fig=idx_fig+1;hold off;
    
     % ver Aceleraciones lineales globales
     figure(idx_fig);
     plot(Acc_G(:,1),'r'); hold on;
     plot(Acc_G(:,2),'g');
     plot(Acc_G(:,3),'b');
     ylabel('m/s^2'); title('Accelerations in the global coordinate frame (G)');
     xlabel('samples');
     legend('a_{x_g}','a_{y_g}','a_{z_g}');
     idx_fig=idx_fig+1;hold off;
     
     % ver Velocidades lineales globales
     figure(idx_fig);
     plot(Velo_G(:,1),'r'); hold on;
     plot(Velo_G(:,2),'g');
     plot(Velo_G(:,3),'b');
     plot(Step_event,zeros(1,size(Step_event,2)),'ro','MarkerSize',6,'MarkerFaceColor',[1 0 0]);
     plot(Step_event_ext,Error_Velo_G(:,1),'rx','MarkerSize',6);
     plot(Step_event_ext,Error_Velo_G(:,2),'gx','MarkerSize',6);
     plot(Step_event_ext,Error_Velo_G(:,3),'bx','MarkerSize',6);
     ylabel('m/s'); title('Integrated linear velocities in the global coordinate frame (G)');
     xlabel('samples');
     legend('v_{x_g}','v_{y_g}','v_{z_g}','Step_event event');
     idx_fig=idx_fig+1;hold off;
     
     % ver Velocidades lineales globales Corregidas
     figure(idx_fig);
     plot(Velo_G_corr(:,1),'r'); hold on;
     plot(Velo_G_corr(:,2),'g');
     plot(Velo_G_corr(:,3),'b');
     plot(Step_event_ext,zeros(1,size(Step_event_ext,2)),'ro','MarkerSize',6,'MarkerFaceColor',[1 0 0]);
     ylabel('m/s'); title('Integrated linear velocities in the global coordinate frame (G) with ZUPT');
     xlabel('samples');
     legend('v_{x_g}','v_{y_g}','v_{z_g}','Step_event event');
     idx_fig=idx_fig+1;hold off;
     
    % ver Positions y Thetas
     figure(idx_fig);
     plot3(Positions(:,2),Positions(:,1),Positions(:,3),'bo-'); hold on; % marcar trayectoria
     plot3(Positions(1,2),Positions(1,1),0,'bs','MarkerSize',8,'MarkerFaceColor',[0 0 1]); % marcar posicion inicial
     plot3(Positions(end,2),Positions(end,1),0,'bo','MarkerSize',8,'MarkerFaceColor',[0 0 1]); % marcar paso final
     %plot(Thetas(:,1),Positions(:,1),'g');
     ylabel('North (m)'); title('SL-ZUPT+Theta: Positions of trayectories in the global coordinate frame (G)');
     xlabel('East (m)');
     axis equal
     idx_fig=idx_fig+1;hold off;
     

end