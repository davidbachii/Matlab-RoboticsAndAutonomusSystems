%% INICIALIZACIÓN DE ROS
% Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP
%de la máquina donde se ejecuta Matlab). Si se está conectado a la misma red, la
%variable ROS_IP no es necesario definirla.
setenv('ROS_MASTER_URI','http://172.29.30.178:11311');
setenv('ROS_IP','172.29.30.11') %Aqui poner la ip de windows
rosshutdown;
rosinit() % Inicialización de ROS en la IP correspondiente
% DECLARACION DE SUSCRIBERS
%Odometria
sub_odom = rossubscriber('/pose', 'nav_msgs/Odometry');

%Laser
sub_laser = rossubscriber('/scan', 'sensor_msgs/LaserScan');

%Sonars
sub_sonar0 = rossubscriber('/sonar_0', 'sensor_msgs/Range');
sub_sonar1 = rossubscriber('/sonar_1', 'sensor_msgs/Range');
sub_sonar2 = rossubscriber('/sonar_2', 'sensor_msgs/Range');
sub_sonar3 = rossubscriber('/sonar_3', 'sensor_msgs/Range');
sub_sonar4 = rossubscriber('/sonar_4', 'sensor_msgs/Range');
sub_sonar5 = rossubscriber('/sonar_5', 'sensor_msgs/Range');
sub_sonar6 = rossubscriber('/sonar_6', 'sensor_msgs/Range');
sub_sonar7 = rossubscriber('/sonar_7', 'sensor_msgs/Range');

% DECLARACION DE PUBLISHERS
%Velocidad
pub_vel = rospublisher('/cmd_vel','geometry_msgs/Twist');
pub_motor = rospublisher('/cmd_motor_state','std_msgs/Int32');

msg_enable_motor = rosmessage (pub_motor);
msg_enable_motor.Data = 1;
send (pub_motor, msg_enable_motor);

% GENERACION de MENSAJES
msg_vel = rosmessage(pub_vel);

% Definimos la periocidad del bucle
r = rateControl(10);

% Nos aseguramos de recibir un mesaje relacionado con el robot
disp('Inicializacion finalizada correctamente');