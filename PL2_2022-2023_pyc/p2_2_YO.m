%% INICIALIZACIÓN DE ROS (COMPLETAR ESPACIOS CON LAS DIRECCIONES IP)
setenv('ROS_MASTER_URI','http://192.168.1.39:11311');
setenv('ROS_IP','192.168.1.36') %Aqui poner la ip de windows
rosinit() % Inicialización de ROS en la IP correspondiente
%% DECLARACIÓN DE VARIABLES NECESARIAS PARA EL CONTROL
MAX_TIME = 1000; %% Numero máximo de iteraciones
medidas = zeros(5,1000);
%% DECLARACIÓN DE SUBSCRIBERS
odom = rossubscriber('/robot0/odom'); % Subscripción a la odometría
receive(odom);
sonar0 = rossubscriber('/robot0/sonar_0', rostype.sensor_msgs_Range);
%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
msg_vel=rosmessage(pub); %% Creamos un mensaje del tipo declarado en "pub" (geometry_msgs/Twist)
msg_sonar0=rosmessage(sonar0);
%% Definimos la periodicidad del bucle (10 hz)
r = robotics.Rate(10);
waitfor(r);
%% Inicializamos variables para el control
D = 1;
i = 0;
msg_sonar0 = receive (sonar0);
Kd = 1.8; 
Ko = 0.18;
dist = 0;
lastpos = odom.LatestMessage.Pose.Pose.Position;
lastdist = dist;
lastdistav = 0;
%% Bucle de control
msg_vel.Linear.X = 1; % velocidad lineal hacia adelante
send(pub, msg_vel); % enviar el mensaje de velocidad al robot
pause(1); % esperar 1 segundo
msg_vel.Linear.X = 0; % velocidad lineal hacia adelante
send(pub, msg_vel);
while (1)
i = i + 1;
pos=odom.LatestMessage.Pose.Pose.Position
%% Calculamos la distancia avanzada y medimos la distancia a la pared
distav = sqrt((lastpos.X - pos.X)^2 + (lastpos.Y - pos.Y)^2)
msg_sonar0 = receive (sonar0);
dist = msg_sonar0.Range_;
if dist>5
 dist = 5;
end
%% Calculamos el error de distancia y orientación
Eori = atan2(abs(dist-lastdist), distav)
Edist = ((dist + 0.105))*cos(Eori) - D;
medidas(1,i)= dist;
medidas(2,i)= lastdist; %% valor anterior de distancia
medidas(3,i)= distav;
medidas(4,i)= Eori;
medidas(5,i)= Edist;
%% Calculamos las consignas de velocidades
consigna_vel_linear = 0.3;
consigna_vel_ang = Kd*Edist + Ko*Eori;
%% Aplicamos consignas de control
msg_vel.Linear.X= consigna_vel_linear;
msg_vel.Linear.Y=0;
msg_vel.Linear.Z=0;
msg_vel.Angular.X=0;
msg_vel.Angular.Y=0;
msg_vel.Angular.Z= consigna_vel_ang;
disp(dist);
% Comando de velocidad
send(pub,msg_vel);
lastpos = pos;
lastdist = dist;
lastvAng = consigna_vel_ang;
lastdistav = distav;
disp("------------------------------------------------------------------------------------------------------------------")
% Temporización del bucle según el parámetro establecido en r
waitfor(r);
if i==MAX_TIME
 break;
end
end
save('medidas.mat','medidas');