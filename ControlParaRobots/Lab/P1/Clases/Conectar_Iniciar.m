%% INICIALIZACIÓN DE ROS
% Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP
%de la máquina donde se ejecuta Matlab). Si se está conectado a la misma red, la
%variable ROS_IP no es necesario definirla.
setenv('ROS_MASTER_URI','http://192.168.0.21:11311'); %ip de ubuntu
setenv('ROS_IP','192.168.0.21') %Aqui poner la ip de windows
rosshutdown;
rosinit() % Inicialización de ROS en la IP correspondiente
% DECLARACION DE SUSCRIBERS
%Odometria
sub_odom = rossubscriber('/robot0/odom', 'nav_msgs/Odometry');

%Laser
sub_laser = rossubscriber('/robot0/laser_1', 'sensor_msgs/LaserScan');

%Sonars
sub_sonar0 = rossubscriber('/robot0/sonar_0', 'sensor_msgs/Range');
sub_sonar1 = rossubscriber('/robot0/sonar_1', 'sensor_msgs/Range');
sub_sonar2 = rossubscriber('/robot0/sonar_2', 'sensor_msgs/Range');
sub_sonar3 = rossubscriber('/robot0/sonar_3', 'sensor_msgs/Range');
sub_sonar4 = rossubscriber('/robot0/sonar_4', 'sensor_msgs/Range');
sub_sonar5 = rossubscriber('/robot0/sonar_5', 'sensor_msgs/Range');
sub_sonar6 = rossubscriber('/robot0/sonar_6', 'sensor_msgs/Range');
sub_sonar7 = rossubscriber('/robot0/sonar_7', 'sensor_msgs/Range');



%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %

%% GENERACIÓN DE MENSAJE
msg=rosmessage(pub); %% Creamos un mensaje del tipo declarado en "pub"
% (geometry_msgs/Twist)

%Rellenamos los campos del mensaje para que el robot avance a 0.2 m/s

% Velocidades lineales en x,y y z (velocidades en y o z no se usan en robots 
% diferenciales y entornos 2D)
msg.Linear.X=0;
msg.Linear.Y=0;
msg.Linear.Z=0;

% Velocidades angulares (en robots diferenciales y entornos 2D solo se utilizará
% el valor Z)
msg.Angular.X=0;
msg.Angular.Y=0;
msg.Angular.Z=0;

%%Mensajes para movimiento angular
%% GENERACIÓN DE MENSAJE
msga=rosmessage(pub); %% Creamos un mensaje del tipo declarado en "pub"
% (geometry_msgs/Twist)

%Rellenamos los campos del mensaje para que el robot avance a 0.2 m/s

% Velocidades lineales en x,y y z (velocidades en y o z no se usan en robots 
% diferenciales y entornos 2D)
msga.Linear.X=0;
msga.Linear.Y=0;
msga.Linear.Z=0;

% Velocidades angulares (en robots diferenciales y entornos 2D solo se utilizará
% el valor Z)
msga.Angular.X=0;
msga.Angular.Y=0;
msga.Angular.Z= 0;

%% Definimos la perodicidad del bucle (10 hz)
r = robotics.Rate(10);

%% Nos aseguramos recibir un mensaje relacionado con el robot "robot0"
pause(1); % Esperamos 1 segundo para asegurarnos que ha llegado algún mensaje odom,
% porque sino ls función strcmp da error al tener uno de los campos vacios.

while (strcmp(sub_odom.LatestMessage.ChildFrameId,'robot0')~=1)
    sub_odom.LatestMessage;

end