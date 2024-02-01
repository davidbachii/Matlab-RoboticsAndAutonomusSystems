avanzarx(3.5);
function avanzarx(distancia_recorrida)

%% INICIALIZACIÓN DE ROS
% Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP
%de la máquina donde se ejecuta Matlab). Si se está conectado a la misma red, la
%variable ROS_IP no es necesario definirla.
setenv('ROS_MASTER_URI','http://172.22.59.110:11311');
setenv('ROS_IP','172.22.20.145') %Aqui poner la ip de windows
rosinit() % Inicialización de ROS en la IP correspondiente
%% DECLARACIÓN DE SUBSCRIBERS
odom=rossubscriber('/robot0/odom'); % Subscripción a la odometría
%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
%% GENERACIÓN DE MENSAJE
msg=rosmessage(pub); %% Creamos un mensaje del tipo declarado en "pub (geometry_msgs/Twist)
% Rellenamos los campos del mensaje para que el robot avance a 0.3 m/s
msg.Linear.X=0.8;

%% Definimos la perodicidad del bucle (10 hz)
r = robotics.Rate(10);
%% Nos aseguramos recibir un mensaje relacionado con el robot "robot0"
pause(1); % Esperamos 1 segundo para asegurarnos que ha llegado algún mensaje odom, porque sino ls función strcmp da error al tener uno de los campos vacios.
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0')~=1)
 odom.LatestMessage;
end

%% Inicializamos la primera posición (coordenadas x,y,z)
initpos=odom.LatestMessage.Pose.Pose.Position;
 

%% Bucle de control infinito
while (1)
%% Obtenemos la posición actual
    pos=odom.LatestMessage.Pose.Pose.Position;
%% Calculamos la distancia euclÃdea que se ha desplazado
    dist=sqrt((initpos.X-pos.X)^2+(initpos.Y-pos.Y)^2);
%% Si el robot se ha desplazado más de un metro detenemos el robot (velocidad lineal 0) y salimos del bucle
    if (dist>distancia_recorrida)
        msg.Linear.X=0;
        % Comando de velocidad
        send(pub,msg);
        % Salimos del bucle de control
        break;
    else
    % Comando de velocidad
        send(pub,msg);
    end
% Temporización del bucle según el parámetro establecido en r
waitfor(r);
end

%% DESCONEXIÓN DE ROS
rosshutdown;

end



