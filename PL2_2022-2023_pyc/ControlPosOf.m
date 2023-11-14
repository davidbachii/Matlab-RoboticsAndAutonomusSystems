%% INICIALIZACIÓN DE ROS
% Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP
%de la máquina donde se ejecuta Matlab). Si se está conectado a la misma red, la
%variable ROS_IP no es necesario definirla.
setenv('ROS_MASTER_URI','http://192.168.0.19:11311')  %Aqui poner la ip de ubuntu 
setenv('ROS_IP','192.168.0.15') %Aqui poner la ip de windows
rosinit % Inicialización de ROS
%% DECLARACIÓN DE SUBSCRIBERS
odom=rossubscriber('/robot0/odom'); % Subscripción a la odometría
%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
%% GENERACIÓN DE MENSAJE
msg=rosmessage(pub) %% Creamos un mensaje del tipo declarado en "pub (geometry_msgs/Twist)

%% Definimos la perodicidad del bucle (10 hz)
r = robotics.Rate(10);
%% Nos aseguramos recibir un mensaje relacionado con el robot "robot0"
pause(1); % Esperamos 1 segundo para asegurarnos que ha llegado algún mensaje odom, porque sino ls función strcmp da error al tener uno de los campos vacios.
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0')~=1)
 odom.LatestMessage
end
% Pedir al usuario las coordenadas de destino
x_destino = input('Introduce la coordenada x de destino: ');
y_destino = input('Introduce la coordenada y de destino: ');
% Establecer el punto de inicio
tic

% Inicializar variables
Kp = 0.5; % Constante de proporcionalidad del control de velocidad
Ka = 1.0; % Constante de proporcionalidad del control de orientación
tolerancia = 0.1; % Tolerancia de distancia y orientación
% Bucle de control
while true
    % Obtener la odometría actual del robot
    
    x_actual = odom.LatestMessage.Pose.Pose.Position.X;
    disp("Eje x:"+x_actual);
    y_actual = odom.LatestMessage.Pose.Pose.Position.Y;
    disp("Eje y:"+y_actual);
    
    pos=odom.LatestMessage.Pose.Pose.Orientation;
    qpos = [pos.W, pos.X, pos.Y, pos.Z];
    % Convertir el resultado anterior en ángulos de Euler en radianes
    [yaw, pitch, roll] = quat2angle(qpos, 'ZYX');
    
    % Calcular el error de distancia y orientación
    DE = sqrt((x_destino - x_actual)^2 + (y_destino - y_actual)^2);
    disp("Error distancia:"+DE);
    OE = atan2(y_destino - y_actual, x_destino - x_actual) - yaw;
    disp("Error orientacion:"+OE);
    
    % Control de velocidad lineal en función del error de distancia
    v_lineal = Kp * DE;
    v_lineal = min(v_lineal, 0.5); % Limitar la velocidad máxima
    disp("Velocidad lineal:"+v_lineal);
    
    % Control de velocidad angular en función del error de orientación
    v_angular = Ka * OE;
    v_angular = min(v_angular, 1.0); % Limitar la velocidad angular máxima
    disp("Velocidad angular:"+v_angular);
    
    
    % Enviar comandos de velocidad al robot
    
    msg.Linear.X = v_lineal;
    msg.Angular.Z = v_angular;
    send(pub,msg);
    
    % Comprobar si se ha llegado a la posición de destino
    if DE < tolerancia && abs(OE) < tolerancia
         % Establecer el punto de finalización y mostrar el tiempo transcurrido
        toc
        disp(toc);
        break;
       
        
    end
    
    % Esperar un tiempo para evitar saturar el sistema
    pause(0.1);
end

% Parar el robot
msg.Linear.X=0;
msg.Angular.Z=0;
send(pub, msg);
%% DESCONEXIÓN DE ROS
rosshutdown;