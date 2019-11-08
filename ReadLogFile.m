function [Acc,Gyr,Magn,Gnss,Wifi,AccX,GyrX]=ReadLogFile(filename,ver,idx_fig)

% Leer y visualiza los datos grabados en un log_file creado con la App android GetSensorData
%
% INPUT:
%   filename:    Nombre del fichero texto
%   ver:         Controla los graficos y datos de salida que nos interesan para la práctica. Opciones: 'smartphone', 'Xsens', 'full'(default)
%   idx_fig:     indice de la figura a partir de la cual abrir figuras
% OUTPUT:
%   Acc,Gyr    Acceleration and angular rate of smartphone sensors (ACCE and GYRO lines) [nx4] 4: 3 axis +timestamp 
%   AccX,GyrX  Acceleration and angular rate of external Xsens sensor (IMUX lines) [nx4]  4: 3 axis + timestamp 
%
%  use example :  
%    > [Acce,Gyro,AcceX,GyroX]=ReadLogFile('\log_files\logfile3_UAH_S4.txt');
%    > [Acce,Gyro,~]=ReadLogFile('\log_files\logfile3_UAH_S4.txt','smartphone');
%    > [~,~,AcceX,GyroX]=ReadLogFile('\log_files\logfile3_UAH_S4.txt','Xsens');


% Read and view the recorded data in a log_file created with the GetSensorData Android App
%
% INPUT:
% filename: Name of the text file
% see: Controls the graphics and output data that interest us for practice. Options: 'smartphone', 'Xsens', 'full' (default)
% idx_fig: index of the figure from which to open figures
% OUTPUT:
% Acc, Gyr Acceleration and angular rate of smartphone sensors (ACCE and GYRO lines) [nx4] 4: 3 axis + timestamp
% AccX, GyrX Acceleration and angular rate of external Xsens sensor (IMUX lines) [nx4] 4: 3 axis + timestamp
%
% use example:
%> [Acce, Gyro, AcceX, GyroX] = ReadLogFile ('\ log_files \ logfile3_UAH_S4.txt');
%> [Acce, Gyro, ~] = ReadLogFile ('\ log_files \ logfile3_UAH_S4.txt', 'smartphone');
%> [~, ~, AcceX, GyroX] = ReadLogFile ('\ log_files \ logfile3_UAH_S4.txt', 'Xsens');

if ~exist('ver','var')
     % if does not exist, so put default value
      ver='full';
end

if ~exist('idx_fig','var')
     % if does not exist, so put default value
      idx_fig=1;
end

%==============================================
% Definir el fichero a leer y leerlo:
% Define the file to read and read it:
%==============================================

% Abrir fichero y volcar contenido en "datos_fichero"
% Open file and dump content in "data_file"

fid = fopen(filename);
datos_fichero=fread(fid);  % leo el fichero y lo cargo todo en datos
			   % I read the file and load it all in data
posfinlineas=find(datos_fichero==10);  % busco saltos de linea (10 = LF(LineFeed),  13=CR(Carrige Return))
					% search line breaks (10 = LF (Line Feed), 13 = CR (Carriage Return))
posfinlineas=[0; posfinlineas]; %
numlineas=length(posfinlineas)-1; % numero de lineas en fichero
				  % number of lines in file
disp(['Number of lines in logfile: ',num2str(numlineas)]);

%==============================================
% Parsear los datos y Grabarlos en un .mat
% Parse the data and save it in a .mat
%==============================================
idx_linea=0;  % reseteo el indice de la linea del fichero
% Reservar memoria
% reset the file line index
% Reserve memory
Posi=ones(numlineas,6)*NaN; index_Posi=1;
Acce=ones(numlineas,6)*NaN; index_Acce=1;
Gyro=ones(numlineas,6)*NaN; index_Gyro=1;
Magn=ones(numlineas,6)*NaN; index_Magn=1;
Pres=ones(numlineas,4)*NaN; index_Pres=1;
Ligh=ones(numlineas,4)*NaN; index_Ligh=1;
Prox=ones(numlineas,4)*NaN; index_Prox=1;
Soun=ones(numlineas,4)*NaN; index_Soun=1;
Ahrs=ones(numlineas,9)*NaN; index_Ahrs=1;
Gnss=ones(numlineas,10)*NaN; index_Gnss=1;
Rfid=ones(numlineas,5)*NaN; index_Rfid=1;% datos RFID con 5 columas: 1)time stamp, 2) id_reader, 3) id_tag, 4) RSS1  y RSS2
					 % RFID data with 5 columns: 1) time stamp, 2) id_reader, 3) id_tag, 4) RSS1 and RSS2
Wifi=ones(numlineas,4)*NaN; index_Wifi=1;
Imul=ones(numlineas,21)*NaN; index_Imul=1;
Imux=ones(numlineas,21)*NaN; index_Imux=1;

eof=false;  datos=[];  tipo='';
while (~eof)
    % Leer una medida %[tipo,datos,eof]=obj.getSiguienteMedida();
    % Read a measure% [type, data, eof] = obj.getNextMeasurement ();
    linea_con_medida=false; %line_with_measurement
    while (~linea_con_medida && ~eof)
        idx_linea=idx_linea+1;
        if ( idx_linea<=numlineas) % hay lineas por procesar
        % there are lines to process
            idx_ini=posfinlineas(idx_linea)+1; % posicion absoluta de principio de linea en 'data'
            % absolute position of beginning of line in 'data'
            idx_fin=posfinlineas(idx_linea+1)-1; % posicion absoluta de final de linea actual
            % absolute position of current end of line
            linea=char(datos_fichero(idx_ini:idx_fin)');
            % disp(['Linea: ',linea]);
            % Procesamos la línea
            % LogFile Data format:
            % We process the line
            % LogFile Data format:
            % Accelerometer data: 	'ACCE;AppTimestamp(s);SensorTimestamp(s);Acc_X(m/s^2);Acc_Y(m/s^2);Acc_Z(m/s^2);Accuracy(integer)'
            % Gyroscope data:     	'GYRO;AppTimestamp(s);SensorTimestamp(s);Gyr_X(rad/s);Gyr_Y(rad/s);Gyr_Z(rad/s);Accuracy(integer)'
            % Magnetometer data:  	'MAGN;AppTimestamp(s);SensorTimestamp(s);Mag_X(uT);;Mag_Y(uT);Mag_Z(uT);Accuracy(integer)'
            % Pressure data:      	'PRES;AppTimestamp(s);SensorTimestamp(s);Pres(mbar);Accuracy(integer)'
            % Light data:         	'LIGH;AppTimestamp(s);SensorTimestamp(s);Light(lux);Accuracy(integer)'
            % Proximity data:     	'PROX;AppTimestamp(s);SensorTimestamp(s);prox(?);Accuracy(integer)'
            % Humidity data:      	'HUMI;AppTimestamp(s);SensorTimestamp(s);humi(%);Accuracy(integer)'
            % Temperature data:   	'TEMP;AppTimestamp(s);SensorTimestamp(s);temp(ÂºC);Accuracy(integer)'
            % Orientation data:   	'AHRS;AppTimestamp(s);SensorTimestamp(s);PitchX(Âº);RollY(Âº);YawZ(Âº);RotVecX();RotVecY();RotVecZ();Accuracy(int)'
            % GNSS/GPS data:        'GNSS;AppTimestamp(s);SensorTimeStamp(s);Latit(º);Long(º);Altitude(m);Bearing(º);Accuracy(m);Speed(m/s);SatInView;SatInUse'
            % WIFI data:          	'WIFI;AppTimestamp(s);SensorTimeStamp(s);Name_SSID;MAC_BSSID;RSS(dBm);'
            % Bluetooth data:     	'BLUE;AppTimestamp(s);Name;MAC_Address;RSS(dBm);'
            % Sound data:         	'SOUN;AppTimestamp(s);RMS;Pressure(Pa);SPL(dB);'
            % RFID Reader data:   	'RFID;AppTimestamp(s);ReaderNumber(int);TagID(int);RSS_A(dBm);RSS_B(dBm);'
            % IMU XSens data:     	'IMUX;AppTimestamp(s);SensorTimestamp(s);Counter;Acc_X(m/s^2);Acc_Y(m/s^2);Acc_Z(m/s^2);Gyr_X(rad/s);Gyr_Y(rad/s);Gyr_Z(rad/s);Mag_X(uT);;Mag_Y(uT);Mag_Z(uT);Roll(Âº);Pitch(Âº);Yaw(Âº);Pressure(mbar);Temp(ÂºC)'
            % IMU LPMS-B data:    	'IMUL;AppTimestamp(s);SensorTimestamp(s);Counter;Acc_X(m/s^2);Acc_Y(m/s^2);Acc_Z(m/s^2);Gyr_X(rad/s);Gyr_Y(rad/s);Gyr_Z(rad/s);Mag_X(uT);;Mag_Y(uT);Mag_Z(uT);Roll(Âº);Pitch(Âº);Yaw(Âº);Pressure(mbar);Temp(ÂºC)'
            % POSI Reference:  		'POSI;AppTimestamp(s);Counter;Latitude(degrees); Longitude(degrees);floor ID(0,1,2..4);Building ID(0,1,2..3);'
            % Note that there are two timestamps:
            %  -'AppTimestamp' is set by the Android App as data is read. It is not representative of when data is actually captured by the sensor (but has a common time reference for all sensors)
            %  -'SensorTimestamp' is set by the sensor itself (the delta_time=SensorTimestamp(k)-SensorTimestamp(k-1) between two consecutive samples is an accurate estimate of the sampling interval). This timestamp is better for integrating inertial data.
            
            % Inicializo la salida de datos. Si no es nada reconocible -> los datos y tipo vacios
            % Initialized data output. If nothing is recognizable -> empty data and type
            datos=[];
            tipo='';
            
            if (numel(linea)>0 && ~strcmp(linea(1),'%') )
                if ( strfind(linea,'POSI'))  % Es una linea de Marca Posicion
						% It is a line of Position Brand
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f')'; %output=textscan(linea,'%*4s;%f;%f;%f;%f;%f')';
                    tipo='POSI';
                end
                if ( strfind(linea,'ACCE'))  % Es una linea de Acelerometro
						% It is an accelerometer line
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f')'; %output=textscan(linea,'%*4s;%f;%f;%f;%f;%f')';
                    tipo='ACCE';
                end
                if ( strfind(linea,'GYRO'))  % Es una linea de Gyroscopo
						% It is a Gyroscopo line
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f')';
                    tipo='GYRO';
                end
                if ( strfind(linea,'MAGN'))  % Es una linea de Magnetometro
						% It is a Magnetometer line
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f')';
                    tipo='MAGN';
                end
                if ( strfind(linea,'PRES'))  % Es una linea de Presion
						% It is a pressure line
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f')';
                    tipo='PRES';
                end
                if ( strfind(linea,'LIGH'))  % Es una linea de Luminosidad. p.ej: LIGH;14;22.370;2565.0;0
						% It is a line of brightness. eg: LIGH; 14; 22,370; 2565.0; 0
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f')';
                    tipo='LIGH';
                end
                if ( strfind(linea,'PROX'))  % Es una linea de Proximidad. p.ej: PROX;0.23;22.370;0.0;0
						% It is a line of Proximity. eg: PROX; 0.23; 22,370; 0.0; 0
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f')';
                    tipo='PROX';
                end
                if ( strfind(linea,'SOUN'))  % Es una linea de Sonido. p.ej.: SOUN;0.200;576.05;0.01758;58.88
						% It is a Sound line. eg: SOUN; 0.200; 576.05; 0.01758; 58.88
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f')';
                    tipo='SOUN';
                end
                if ( strfind(linea,'AHRS'))  % Es una linea de Orientacion
						% It is an orientation line
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f;%f;%f;%f')';
                    tipo='AHRS';
                end
                if ( strfind(linea,'GNSS'))  % Es una linea de GPS
                    % 'GNSS;Timestamp(s);SensorTimeStamp(s);Latitude(º);Longitude(º);Altitude(m);Bearing(º),Accuracy(m),Speed(m/s),satellites_in_view;num_satellites_in_use'
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f')';
                    tipo='GNSS';
                end
                if ( strfind(linea,'RFID'))  % Es una linea de RFID. p.ej.: "RFID;22.365;29;67937;80;80"
						% It is an RFID line. eg: "RFID; 22,365; 29; 67937; 80; 80"
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f')';
                    tipo='RFID';
                end
                if ( strfind(linea,'WIFI'))  % Es una linea de WIFI.
                    % Formato nuevo WIFI data: 'WIFI;AppTimestamp(s);SensorTimeStamp(s);Name_SSID;MAC_BSSID;RSS(dBm);'
                    % p.ej.: "WIFI;22.365;8051.234;portal-csic;00:0b:86:27:36:c1;-71"
                    % New format WIFI data: 'WIFI; AppTimestamp (s); SensorTimeStamp (s); Name_SSID; MAC_BSSID; RSS (dBm);'
                    % eg: "WIFI; 22,365; 8051,234; portal-csic; 00: 0b: 86: 27: 36: c1; -71"
                    cell_array=textscan(linea,'%*s %f %f %*s %s %f','delimiter',';');
                    datos(1)=cell_array{1,1}; % timestamp
                    datos(2)=cell_array{1,2}; % Sensortimestamp
                    datos(4)=cell_array{1,4}; % RSS
                    MAC_str=cell_array{1,3}{1,1}; % MAC
                    MAC_dec_array=sscanf(MAC_str,'%x:%x:%x:%x:%x:%x'); % quitar ":" y convertir a numero
                    MAC_dec=MAC_dec_array(1)*256^5+MAC_dec_array(2)*256^4+MAC_dec_array(3)*256^3+MAC_dec_array(4)*256^2+MAC_dec_array(5)*256+MAC_dec_array(6);
                    datos(3)=MAC_dec;
                    tipo='WIFI';
                    % disp(['Linea: ',linea]);
                end
                if ( strfind(linea,'IMUL'))  % Es una linea de IMUL. p.ej.: "IMUL;0.929;1907.2437;11782;0.22992;-0.15328;9.31184;-0.01853;0.01550;0.00004;-24.02985;56.41791;22.50000;-1.28843;1.03222;121.61204;936.870;0.00"
                    % 21 campos: Timestamp(s);SensorTimestamp(s);Counter;Acc_X(m/s^2);Acc_Y(m/s^2);Acc_Z(m/s^2);Gyr_X(rad/s);Gyr_Y(rad/s);Gyr_Z(rad/s);Mag_X(uT);;Mag_Y(uT);Mag_Z(uT);Roll(º);Pitch(º);Yaw(º);Quat1;Quat2;Quat3;Quat4;Pressure(mbar);Temp(ºC)'");
                    % It is an IMUL line. eg: "IMUL; 0.929; 1907.2437; 11782; 0.22992; -0.15328; 9.31184; -0.01853; 0.01550; 0.00004; -24.02985; 56.41791; 22.50000; -1.28843; 1.03222; 121.61204; 936.870; 0.00 "
                    % 21 fields: Timestamp (s); SensorTimestamp (s); Counter; Acc_X (m / s ^ 2); Acc_Y (m / s ^ 2); Acc_Z (m / s ^ 2);                         Gyr_X (rad / s); Gyr_Y (rad / s); Gyr_Z (rad / s); Mag_X(uT) ;; Mag_Y (uT); Mag_Z (uT); Roll (º); Pitch (º);                            Yaw (º); Quat1; Quat2; Quat3; Quat4; Pressure (mbar); Temp (ºC) '") ;

                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f')';
                    % Calibro magnetometro por hard iron effects:
		            % Calibro magnetometer by hard iron effects:
                    datos(10:12)=datos(10:12)-[-22.7386, 44.0594, 59.0431]; % Quitar cuando calibre internamente en la flash del LPMS-B
									    % Remove when calibrating internally on the LPMS-B flash
                    tipo='IMUL';
                end
                if ( strfind(linea,'IMUX'))  % Es una linea de IMUX. p.ej.: "IMUL;0.929;1907.2437;11782;0.22992;-0.15328;9.31184;-0.01853;0.01550;0.00004;-24.02985;56.41791;22.50000;-1.28843;1.03222;121.61204;936.870;0.00"
                    % 21 campos: Timestamp(s);SensorTimestamp(s);Counter;Acc_X(m/s^2);Acc_Y(m/s^2);Acc_Z(m/s^2);Gyr_X(rad/s);Gyr_Y(rad/s);Gyr_Z(rad/s);Mag_X(uT);;Mag_Y(uT);Mag_Z(uT);Roll(º);Pitch(º);Yaw(º);Quat1;Quat2;Quat3;Quat4;Pressure(mbar);Temp(ºC)'");
                    % It is an IMUX line. eg: "IMUL; 0.929; 1907.2437; 11782; 0.22992; -0.15328; 9.31184; -0.01853; 0.01550; 0.00004; -24.02985; 56.41791; 22.50000; -1.28843; 1.03222; 121.61204; 936.870; 0.00 "
                    % 21 fields: Timestamp (s); SensorTimestamp (s); Counter; Acc_X (m / s ^ 2); Acc_Y (m / s ^ 2); Acc_Z (m / s ^ 2);                     Gyr_X (rad / s); Gyr_Y (rad / s); Gyr_Z (rad / s); Mag_X(uT) ;; Mag_Y (uT); Mag_Z (uT); Roll (º); Pitch (º); Yaw (º)                    ; Quat1; Quat2; Quat3; Quat4; Pressure (mbar); Temp (ºC) '") ;
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f')';
                    tipo='IMUX';
                end
            end
            if ~isempty(datos)
                linea_con_medida=true; % ya hemos cogido una medida buena (el resto son comentarios)
                % we have already taken a good measure (the rest are comments)
                %disp(['tipo: ',tipo,', Datos: ',num2str(datos),]);
		        % disp (['type:', type, ', Data:', num2str (data),]);
            end
        else
            eof=true;
            disp('No more to read. End of Logfile');
        end
    end
    if eof
        break
    end
    % Almacenar el tipo de medida en la matriz corrrespondiente:
    % Store the type of measurement in the corresponding matrix:
    if strcmp(tipo,'POSI')
        Posi(index_Posi,1:6)=datos;   index_Posi=index_Posi+1;
    end
    if strcmp(tipo,'ACCE')
        Acce(index_Acce,1:6)=datos;   index_Acce=index_Acce+1;
    end
    if strcmp(tipo,'GYRO')
        Gyro(index_Gyro,1:6)=datos;   index_Gyro=index_Gyro+1;
    end
    if strcmp(tipo,'MAGN')
        Magn(index_Magn,1:6)=datos;   index_Magn=index_Magn+1;
    end
    if strcmp(tipo,'PRES')
        Pres(index_Pres,1:4)=datos;   index_Pres=index_Pres+1;
    end
    if strcmp(tipo,'LIGH')
        Ligh(index_Ligh,1:4)=datos;   index_Ligh=index_Ligh+1;
    end
    if strcmp(tipo,'PROX')
        Prox(index_Prox,1:4)=datos;   index_Prox=index_Prox+1;
    end
    if strcmp(tipo,'SOUN')
        Soun(index_Soun,1:4)=datos;   index_Soun=index_Soun+1;
    end
    if strcmp(tipo,'AHRS')
        Ahrs(index_Ahrs,1:9)=datos;   index_Ahrs=index_Ahrs+1;
    end
    if strcmp(tipo,'GNSS')
        Gnss(index_Gnss,1:10)=datos;   index_Gnss=index_Gnss+1;
    end
    if strcmp(tipo,'RFID')
        Rfid(index_Rfid,1:5)=datos;   index_Rfid=index_Rfid+1;
    end
    if strcmp(tipo,'WIFI')
        Wifi(index_Wifi,1:4)=datos;   index_Wifi=index_Wifi+1;
    end
    if strcmp(tipo,'IMUL')
        Imul(index_Imul,1:21)=datos;   index_Imul=index_Imul+1;
    end
    if strcmp(tipo,'IMUX')
        Imux(index_Imux,1:21)=datos;   index_Imux=index_Imux+1;
    end
end
% Recortar arrays datos
% Trim data arrays
Posi=Posi(1:index_Posi-1,:);
Acce=Acce(1:index_Acce-1,:);
Gyro=Gyro(1:index_Gyro-1,:);
Magn=Magn(1:index_Magn-1,:);
Pres=Pres(1:index_Pres-1,:);
Ligh=Ligh(1:index_Ligh-1,:);
Prox=Prox(1:index_Prox-1,:);
Soun=Soun(1:index_Soun-1,:);
Ahrs=Ahrs(1:index_Ahrs-1,:);
Gnss=Gnss(1:index_Gnss-1,:);
Rfid=Rfid(1:index_Rfid-1,:);
Wifi=Wifi(1:index_Wifi-1,:);
Imul=Imul(1:index_Imul-1,:);
Imux=Imux(1:index_Imux-1,:);

% grabar en .mat
% record in .mat
save([filename(1:end-4),'.mat'],'Posi','Acce','Gyro','Magn','Pres','Ligh','Prox','Soun','Ahrs','Gnss','Rfid','Wifi','Imul','Imux');

%==============================================
% Prepara salida sencilla para practica: [nx4]  timestamp, axis1, axis2, axis3
% Prepare simple output for practice: [nx4] timestamp, axis1, axis2, axis3
%==============================================
Acc=[]; Gyr=[];
AccX=[]; GyrX=[];
% Subtract initial timestamp from all data, so that it starts at 0
if (strcmp(ver,'full') || strcmp(ver,'Xsens'))
   AccX=Imux(:,[4:6,2]);  AccX(:,4)=AccX(:,4)-Imux(1,2);   % Acc data
   GyrX=Imux(:,[7:9,2]);  GyrX(:,4)=GyrX(:,4)-Imux(1,2);   % Gyr data 
end
if (strcmp(ver,'full') || strcmp(ver,'smartphone'))
   Acc=Acce(:,[3:5,2]);  Acc(:,4)=Acc(:,4)-Acce(1,2);  % Acc data
   Gyr=Gyro(:,[3:5,2]);  Gyr(:,4)=Gyr(:,4)-Gyro(1,2);  % Gyr data 
end


%==============================================
% Visualizarlos
% Display them
%==============================================

if ~isempty(Posi) 
    figure(idx_fig); idx_fig=idx_fig+1;
    tiempos_Posi=Posi(:,1);
    plot(tiempos_Posi,0,'m*');
    xlabel('time(s)');
    for i=1:length(Posi), text(tiempos_Posi(i),0.1,num2str(i)); end
    title('POSI marks');
end

if ~isempty(Acce) && (strcmp(ver,'full') || strcmp(ver,'smartphone'))
    figure(idx_fig); idx_fig=idx_fig+1;
    Acce_mag=sqrt(sum(Acce(:,3:5).^2,2));
    tiempos=Acce(:,2)-Acce(1,2);
    plot(tiempos,Acce(:,3),'r-'); hold on;
    plot(tiempos,Acce(:,4),'g-'); plot(tiempos,Acce(:,5),'b-');
    plot(tiempos,Acce_mag,'k-'); hold off;
    xlabel('time(s)'); ylabel('Acceleration(m/s^2)');
    freq_Acce=(size(Acce,1)-1)/(Acce(end,1)-Acce(1,1));
    title(['Range:[-39.24:39.24] m/s^2, Resolution: 0.00981 m/s^2 (13 bits), Freq.: ',num2str(freq_Acce),' Hz'])
    legend({'Acc_x','Acc_y','Acc_z','Acc_{mag}'});
end

if ~isempty(Gyro) && (strcmp(ver,'full') || strcmp(ver,'smartphone'))
    figure(idx_fig); idx_fig=idx_fig+1;
    Gyro_mag=sqrt(Gyro(:,3).^2+Gyro(:,4).^2+Gyro(:,5).^2);
    tiempos=Gyro(:,2)-Gyro(1,2);
    plot(tiempos,Gyro(:,3),'r-'); hold on;
    plot(tiempos,Gyro(:,4),'g-'); plot(tiempos,Gyro(:,5),'b-');
    plot(tiempos,Gyro_mag,'k-'); hold off;
    xlabel('time(s)'); ylabel('Angular Rate(rad/s)');
    freq_Gyro=(size(Gyro,1)-1)/(Gyro(end,1)-Gyro(1,1));
    title(['Range: [-34.9:34.9] rad/s, Resolution: 0.0011 rad/s (16 bits), Freq: ',num2str(freq_Gyro),' Hz'])
    legend({'Gyr_x','Gyr_y','Gyr_z','Gyr_{mag}'});
end

if ~isempty(Magn) && strcmp(ver,'full') 
    figure(idx_fig); idx_fig=idx_fig+1;
    Magn_mag=sqrt(Magn(:,3).^2+Magn(:,4).^2+Magn(:,5).^2);
    tiempos=Magn(:,2)-Magn(1,2);
    plot(tiempos,Magn(:,3),'r-'); hold on;
    plot(tiempos,Magn(:,4),'g-'); plot(tiempos,Magn(:,5),'b-');
    plot(tiempos,Magn_mag,'k-'); hold off;
    xlabel('time(s)'); ylabel('Magnetic Field(uT - microTeslas)');
    freq_Magn=(size(Magn,1)-1)/(Magn(end,1)-Magn(1,1));
    title(['Range:[-2000:2000] uT (on Earth typically 25-65 uT), Resolution: 0.0625 uT (16 bits), Freq: ',num2str(freq_Magn),' Hz'])
    legend({'Mag_x','Mag_y','Mag_z','Mag_{mag}'});
end

if ~isempty(Pres) && strcmp(ver,'full')
    figure(idx_fig); idx_fig=idx_fig+1;
    tiempos=Pres(:,2)-Pres(1,2);
    plot(tiempos,Pres(:,3),'r-'); hold on;
    xlabel('time(s)'); ylabel('Pressure (mbar)'); hold off;
    freq_Pres=(size(Pres,1)-1)/(Pres(end,1)-Pres(1,1));
    title(['Range:[625-1250] mbar, Resolution: 0.01 mbar (16 bits), Frequency: ',num2str(freq_Pres),' Hz'])
    legend({'Pressure'});
end

if ~isempty(Ligh) && strcmp(ver,'full')
    figure(idx_fig); idx_fig=idx_fig+1;
    tiempos=Ligh(:,2)-Ligh(1,2);
    plot(tiempos,Ligh(:,3),'r-'); hold on;
    xlabel('time(s)'); ylabel('Light (lux)'); hold off;
    title(['Range: [0:208076 lux] (>1000 outdoors), Resolution: 58 lux (12 bits), Freq.: ',num2str(size(Ligh,1)/Ligh(end,1)),' Hz'])
    legend({'Light'});
end

if ~isempty(Prox) && strcmp(ver,'full')
    figure(idx_fig); idx_fig=idx_fig+1;
    tiempos=Prox(:,2)-Prox(1,2);
    plot(tiempos,Prox(:,3),'r-'); hold on;
    xlabel('time(s)'); ylabel('Prox (a.u)'); hold off;
    title(['Range: [0 o 8] , Freq.: ',num2str(size(Prox,1)/Prox(end,1)),' Hz'])
    legend({'Proximity'});
end

if ~isempty(Ahrs) && (strcmp(ver,'full') )
    figure(idx_fig); idx_fig=idx_fig+1;
    tiempos=Ahrs(:,2)-Ahrs(1,2);
    plot(tiempos,Ahrs(:,3),'r-'); hold on;
    plot(tiempos,Ahrs(:,4),'g-'); plot(tiempos,Ahrs(:,5),'b-'); hold off;
    xlabel('time(s)'); ylabel('Ahrs (º)');
    title(['Range:[?]  , Resolution: ? (? bits), Freq: ',num2str(size(Ahrs,1)/Ahrs(end,1)),' Hz'])
    legend({'Ahrs_x','Ahrs_y','Ahrs_z'});
end

if ~isempty(Soun) && strcmp(ver,'full')
    figure(idx_fig); idx_fig=idx_fig+1;
    subplot(3,1,1); plot(Soun(:,1),Soun(:,2),'r-');  ylabel('RMS');
    title(['Sound, Freq.: ',num2str(size(Soun,1)/Soun(end,1)),' Hz']);
    subplot(3,1,2); plot(Soun(:,1),Soun(:,3),'g-'); ylabel('Pressure(Pa)');
    subplot(3,1,3); plot(Soun(:,1),Soun(:,4),'b-');xlabel('time(s)'); ylabel('SPL(dB)');
end

if ~isempty(Wifi)  && strcmp(ver,'full')
    figure(idx_fig); idx_fig=idx_fig+1;
    plot(Wifi(:,1),Wifi(:,4),'r.'); hold on;
    xlabel('time(s)'); ylabel('RSS (dBm)'); hold off;
    title(['Wifi, Freq.: ',num2str(size(Wifi,1)/Wifi(end,1)),' Hz'])
    legend({'Wifi'});
end

if ~isempty(Rfid)  % datos RFID con 4 columas: 1)time stamp, 2) id_reader*10, 3) id_tag, 4) RSS (los RSS estan desagragados en "procesa_linea")
    tag_id=186478;
    rss_min=40;  % para axis
    figure(idx_fig); idx_fig=idx_fig+1;
    tiempo_max=max(Rfid(:,1));
    subplot(4,1,1);
    idx_pareja_lectorA_tag=find(Rfid(:,2)==280  & Rfid(:,3)==tag_id);
    idx_pareja_lectorB_tag=find(Rfid(:,2)==281  & Rfid(:,3)==tag_id);
    if ~isempty(idx_pareja_lectorA_tag)
        plot(Rfid(idx_pareja_lectorA_tag,1),Rfid(idx_pareja_lectorA_tag,4),'r.-'); hold on;
        plot(Rfid(idx_pareja_lectorB_tag,1),Rfid(idx_pareja_lectorB_tag,4),'b.-');
        legend({'280','281'});
    end
    title(['Tag RFID: ',num2str(tag_id)]); axis([0 tiempo_max rss_min 120]);
    ylabel('RSS 28x (dBm)'); hold off;
    subplot(4,1,2);
    idx_pareja_lectorA_tag=find(Rfid(:,2)==290  & Rfid(:,3)==tag_id);
    idx_pareja_lectorB_tag=find(Rfid(:,2)==291  & Rfid(:,3)==tag_id);
    if ~isempty(idx_pareja_lectorA_tag)
        plot(Rfid(idx_pareja_lectorA_tag,1),Rfid(idx_pareja_lectorA_tag,4),'r.-'); hold on;
        plot(Rfid(idx_pareja_lectorB_tag,1),Rfid(idx_pareja_lectorB_tag,4),'b.-');
        legend({'290','291'});
    end
    ylabel('RSS 29x (dBm)'); axis([0 tiempo_max rss_min 120]); hold off;
    subplot(4,1,3);
    idx_pareja_lectorA_tag=find(Rfid(:,2)==300  & Rfid(:,3)==tag_id);
    idx_pareja_lectorB_tag=find(Rfid(:,2)==301  & Rfid(:,3)==tag_id);
    if ~isempty(idx_pareja_lectorA_tag)
        plot(Rfid(idx_pareja_lectorA_tag,1),Rfid(idx_pareja_lectorA_tag,4),'r.-'); hold on;
        plot(Rfid(idx_pareja_lectorB_tag,1),Rfid(idx_pareja_lectorB_tag,4),'b.-');
        legend({'300','301'});
    end
    ylabel('RSS 30x (dBm)'); axis([0 tiempo_max rss_min 120]); hold off;
    subplot(4,1,4);
    idx_pareja_lectorA_tag=find(Rfid(:,2)==310  & Rfid(:,3)==tag_id);
    idx_pareja_lectorB_tag=find(Rfid(:,2)==311  & Rfid(:,3)==tag_id);
    if ~isempty(idx_pareja_lectorA_tag)
        plot(Rfid(idx_pareja_lectorA_tag,1),Rfid(idx_pareja_lectorA_tag,4),'r.-'); hold on;
        plot(Rfid(idx_pareja_lectorB_tag,1),Rfid(idx_pareja_lectorB_tag,4),'b.-');
        legend({'310','311'});
    end
    xlabel('time(s)'); ylabel('RSS 31x (dBm)'); axis([0 tiempo_max rss_min 120]); hold off;
    
end

if ~isempty(Gnss) && strcmp(ver,'full')
    figure(idx_fig); idx_fig=idx_fig+1;
    subplot(4,2,1);
    plot(Gnss(:,1),Gnss(:,3),'r-');ylabel('Lat(º)');
    title(['GNSS data - Freq: ',num2str(size(Gnss,1)/Gnss(end,1)),' Hz']);
    subplot(4,2,2);
    plot(Gnss(:,1),Gnss(:,4),'r-'); ylabel('Long(º)');
    title(['Resolution=[1e-4º, 1.1 m, 0.3 m/s] ']);
    subplot(4,2,3);
    plot(Gnss(:,1),Gnss(:,5),'r-'); ylabel('Height(m)');
    subplot(4,2,4);
    plot(Gnss(:,1),Gnss(:,7),'r-'); ylabel('Accuracy(m)');
    subplot(4,2,5);
    plot(Gnss(:,1),Gnss(:,6),'r-'); ylabel('Bearing(º)');
    subplot(4,2,6);
    plot(Gnss(:,1),Gnss(:,8),'r-'); ylabel('Speed(m/s)');
    subplot(4,2,7);
    plot(Gnss(:,1),Gnss(:,9),'r-'); ylabel('# Sat in View'); xlabel('time(s)');
    subplot(4,2,8);
    plot(Gnss(:,1),Gnss(:,10),'r-'); ylabel('# Sat in Use'); xlabel('time(s)');
end



if ~isempty(Imux) && (strcmp(ver,'full') || strcmp(ver,'Xsens')) % -----------Comienzan los datos de IMUX--------------
								% ----------- IMUX data begins --------------
    figure(idx_fig); idx_fig=idx_fig+1;
    % 21 campos: Timestamp(s);SensorTimestamp(s);Counter;Acc_X(m/s^2);Acc_Y(m/s^2);Acc_Z(m/s^2);Gyr_X(rad/s);Gyr_Y(rad/s);Gyr_Z(rad/s);Mag_X(uT);;Mag_Y(uT);Mag_Z(uT);Roll(º);Pitch(º);Yaw(º);Pressure(mbar);Temp(ºC)'");
    % indices de Imul: Acc en(4:6), Gyr en(7:9), Mag en(10:12), Euler en(13:15), Quat(16:19),
    %                  Pres en 20 y Temp en 21
    % 21 fields: Timestamp (s); SensorTimestamp (s); Counter; Acc_X (m / s ^ 2); Acc_Y (m / s ^ 2); Acc_Z (m / s ^ 2); Gyr_X (rad / s); Gyr_Y (rad / s); Gyr_Z (rad / s); Mag_X (uT) ;; Mag_Y (uT); Mag_Z(uT); Roll (º); Pitch (º); Yaw (º); Pressure (mbar); Temp (ºC) '");
    % Imul indices: Acc in (4: 6), Gyr in (7: 9), Mag in (10:12), Euler in (13:15), Quat (16:19),
    % Pres in 20 and Temp in 21

    Acce_mag=sqrt(Imux(:,4).^2+Imux(:,5).^2+Imux(:,6).^2);
    tiempos=Imux(:,2)-Imux(1,2);
    plot(tiempos,Imux(:,4),'r-'); hold on;
    plot(tiempos,Imux(:,5),'g-'); plot(tiempos,Imux(:,6),'b-');
    plot(tiempos,Acce_mag,'k-'); hold off;
    xlabel('time(s)'); ylabel('Acceleration(m/s^2)');
    freq_Acce=(size(Imux,1)-1)/(Imux(end,2)-Imux(1,2));
    title(['IMU XSENS, Acceleration, Freq.: ',num2str(freq_Acce),' Hz'])
    legend({'Acc_x','Acc_y','Acc_z','|Acc|'});
    
    figure(idx_fig); idx_fig=idx_fig+1;
    Gyro_mag=sqrt(Imux(:,7).^2+Imux(:,8).^2+Imux(:,9).^2);
    tiempos=Imux(:,2)-Imux(1,2);
    plot(tiempos,Imux(:,7),'r-'); hold on;
    plot(tiempos,Imux(:,8),'g-'); plot(tiempos,Imux(:,9),'b-');
    plot(tiempos,Gyro_mag,'k-'); hold off;
    xlabel('time(s)'); ylabel('Angular Rate(rad/s)');
    freq_Gyro=(size(Imux,1)-1)/(Imux(end,2)-Imux(1,2));
    title(['IMU XSENS, Gyroscope, Freq: ',num2str(freq_Gyro),' Hz'])
    legend({'Gyr_x','Gyr_y','Gyr_z','|Gyr|'});
    
%     figure(idx_fig); idx_fig=idx_fig+1;
%     Magn_mag=sqrt(Imux(:,10).^2+Imux(:,11).^2+Imux(:,12).^2);
%     tiempos=Imux(:,2)-Imux(1,2);
%     plot(tiempos,Imux(:,10),'r-'); hold on;
%     plot(tiempos,Imux(:,11),'g-'); plot(tiempos,Imux(:,12),'b-');
%     plot(tiempos,Magn_mag,'k-'); hold off;
%     xlabel('time(s)'); ylabel('Magnetic Field(uTeslas)');
%     freq_Magn=(size(Imux,1)-1)/(Imux(end,2)-Imux(1,2));
%     title(['IMU XSENS, Magnetometer, Freq: ',num2str(freq_Magn),' Hz'])
%     legend({'Mag_x','Mag_y','Mag_z','|Mag|'});
%     
%     figure(idx_fig); idx_fig=idx_fig+1;
%     contador=Imux(:,3);
%     idx_fault=find(contador<0);
%     contador(idx_fault)=contador(idx_fault)+2^16;  % corrijo por si contador es short (2 bytes) y hay cambio de 32767 a -32768
%     contador=contador-Imux(1,3);
%     SensorTimeStamp=Imux(:,2)-Imux(1,2);
%     Imux(1,1)=0; TimeStamp=Imux(:,1)-Imux(1,1);
%     plot(contador,SensorTimeStamp,'rx-'); hold on;
%     plot(contador,TimeStamp,'b.-');
%     plot(contador(1:end-1),SensorTimeStamp(2:end)-SensorTimeStamp(1:end-1),'k.-');   hold off;
%     xlabel('counter (number of packets)'); ylabel('Time (s)');
%     lost_packets=sum((contador(2:end)-contador(1:end-1))>1);
%     title(['IMU XSENS, Sampling. Lost packets:',num2str(lost_packets)]);
%     legend({'counter vs SensorTimeStamp','counter vs TimeStamp','Delta SensorTimeStamp (should be constant at 0.01 s)'});
%     
%     figure(idx_fig); idx_fig=idx_fig+1;
%     tiempos=Imux(:,2)-Imux(1,2);
%     plot(tiempos,Imux(:,13),'r-'); hold on;
%     plot(tiempos,Imux(:,14),'g-'); plot(tiempos,Imux(:,15),'b-'); hold off;
%     xlabel('time(s)'); ylabel('Orientation (º)');
%     freq_Euler=(size(Imux,1)-1)/(Imux(end,1)-Imux(1,1));
%     title(['IMU XSENS, Euler Orientation, Frequency: ',num2str(freq_Euler),' Hz'])
%     legend({'Roll','Pitch','Yaw'});
%     
%     figure(idx_fig); idx_fig=idx_fig+1;
%     tiempos=Imux(:,2)-Imux(1,2);
%     plot(tiempos,Imux(:,16),'r-'); hold on;
%     plot(tiempos,Imux(:,17),'g-'); plot(tiempos,Imux(:,18),'b-');  plot(tiempos,Imux(:,19),'k-'); hold off;
%     xlabel('time(s)'); ylabel('Quaternions');
%     freq_Euler=(size(Imux,1)-1)/(Imux(end,1)-Imux(1,1));
%     title(['IMU XSENS,  Quaternions, Frequency: ',num2str(freq_Euler),' Hz'])
%     legend({'q1','q2','q3','q4'});
%    
%     figure(idx_fig); idx_fig=idx_fig+1;
%     tiempos=Imux(:,2)-Imux(1,2);
%     subplot(2,1,1);
%     plot(tiempos,Imux(:,20),'b-');  ylabel('Pressure (Pa)');
%     freq_PRESSURE=(size(Imux,1)-1)/(Imux(end,1)-Imux(1,1));
%     title(['IMU XSENS, Pressure & Temperature, Frequency: ',num2str(freq_PRESSURE),' Hz'])
%     subplot(2,1,2);
%     plot(tiempos,Imux(:,21),'r-'); ylabel('Temperature (ºC)');
%     xlabel('time(s)');



% figure (idx_fig); idx_fig = idx_fig + 1;
% Magn_mag = sqrt (Imux (:, 10). ^ 2 + Imux (:, 11). ^ 2 + Imux (:, 12). ^ 2);
% times = Imux (:, 2) -Imux (1,2);
% plot (times, Imux (:, 10), 'r-'); hold on;
% plot (times, Imux (:, 11), 'g-'); plot (times, Imux (:, 12), 'b-');
% plot (times, Magn_mag, 'k-'); hold off;
% xlabel ('time (s)'); ylabel ('Magnetic Field (uTeslas)');
% freq_Magn = (size (Imux, 1) -1) / (Imux (end, 2) -Imux (1,2));
% title (['IMU XSENS, Magnetometer, Freq:', num2str (freq_Magn), 'Hz'])
% legend ({'Mag_x', 'Mag_y', 'Mag_z', '| Mag |'});
%
% figure (idx_fig); idx_fig = idx_fig + 1;
% counter = Imux (:, 3);
% idx_fault = find (counter <0);
% counter (idx_fault) = counter (idx_fault) + 2 ^ 16; % correct if the counter is short (2 bytes) and there is change from 32767 to -32768
% counter = Imux-counter (1,3);
% SensorTimeStamp = Imux (:, 2) -Imux (1,2);
% Imux (1,1) = 0; TimeStamp = Imux (:, 1) -Imux (1,1);
% plot (counter, SensorTimeStamp, 'rx-'); hold on;
% plot (counter, TimeStamp, 'b.-');
% plot (counter (1: end-1), SensorTimeStamp (2: end) -SensorTimeStamp (1: end-1), 'k.-'); hold off;
% xlabel ('counter (number of packets)'); ylabel ('Time (s)');
% lost_packets = sum ((counter (2: end) -counter (1: end-1))> 1);
% title (['IMU XSENS, Sampling. Lost packets:', num2str (lost_packets)]);
% legend ({'counter vs SensorTimeStamp', 'counter vs TimeStamp', 'Delta SensorTimeStamp (should be constant at 0.01 s)'});
%
% figure (idx_fig); idx_fig = idx_fig + 1;
% times = Imux (:, 2) -Imux (1,2);
% plot (times, Imux (:, 13), 'r-'); hold on;
% plot (times, Imux (:, 14), 'g-'); plot (times, Imux (:, 15), 'b-'); hold off;
% xlabel ('time (s)'); ylabel ('Orientation (º)');
% freq_Euler = (size (Imux, 1) -1) / (Imux (end, 1) -Imux (1,1));
% title (['IMU XSENS, Euler Orientation, Frequency:', num2str (freq_Euler), 'Hz'])
% legend ({'Roll', 'Pitch', 'Yaw'});
%
% figure (idx_fig); idx_fig = idx_fig + 1;
% times = Imux (:, 2) -Imux (1,2);
% plot (times, Imux (:, 16), 'r-'); hold on;
% plot (times, Imux (:, 17), 'g-'); plot (times, Imux (:, 18), 'b-'); plot (times, Imux (:, 19), 'k-'); hold off;
% xlabel ('time (s)'); ylabel ('Quaternions');
% freq_Euler = (size (Imux, 1) -1) / (Imux (end, 1) -Imux (1,1));
% title (['IMU XSENS, Quaternions, Frequency:', num2str (freq_Euler), 'Hz'])
% legend ({'q1', 'q2', 'q3', 'q4'});
%
% figure (idx_fig); idx_fig = idx_fig + 1;
% times = Imux (:, 2) -Imux (1,2);
% subplot (2,1,1);
% plot (times, Imux (:, 20), 'b-'); ylabel ('Pressure (Pa)');
% freq_PRESSURE = (size (Imux, 1) -1) / (Imux (end, 1) -Imux (1,1));
% title (['IMU XSENS, Pressure & Temperature, Frequency:', num2str (freq_PRESSURE), 'Hz'])
% subplot (2,1,2);
% plot (times, Imux (:, 21), 'r-'); ylabel ('Temperature (ºC)');
    
end


