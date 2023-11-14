%% INICIALIZACIÓN DE ROS
% Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP
%de la máquina donde se ejecuta Matlab). Si se está conectado a la misma red, la
%variable ROS_IP no es necesario definirla.
setenv('ROS_MASTER_URI','http://192.168.0.15:11311');
setenv('ROS_IP','192.168.0.14') %Aqui poner la ip de windows
rosshutdown;
rosinit() % Inicialización de ROS en la IP correspondiente
%% DECLARACIÓN DE SUBSCRIBERS
odom=rossubscriber('/robot0/odom'); % Subscripción a la odometría
sonarfront = rossubscriber('/robot0/sonar_0'); 
%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); 

  mapear();



function avanzar(distancia_recorrida)
rosshutdown;
%% INICIALIZACIÓN DE ROS
% Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP
%de la máquina donde se ejecuta Matlab). Si se está conectado a la misma red, la
%variable ROS_IP no es necesario definirla.
setenv('ROS_MASTER_URI','http://192.168.0.15:11311');
setenv('ROS_IP','192.168.0.14') %Aqui poner la ip de windows
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

function paredes = detectarParedes()
    rosshutdown;
    paredes = [false, false, false, false];
    
    %% INICIALIZACIÓN DE ROS (COMPLETAR ESPACIOS CON LAS DIRECCIONES IP)
    setenv('ROS_MASTER_URI','http://192.168.0.15:11311');
    setenv('ROS_IP','192.168.0.14') %Aqui poner la ip de windows
    rosinit() % Inicialización de ROS en la IP correspondiente
    sonarfront = rossubscriber('/robot0/sonar_2');  
    sonarback = rossubscriber('/robot0/sonar_6');
    sonarback2 = rossubscriber('/robot0/sonar_7');
    sonarder = rossubscriber('/robot0/sonar_5');
    sonarizq = rossubscriber('/robot0/sonar_0');
    % Leer el mensaje del sensor de sonar
    sonarMsgF = receive(sonarfront, 1);
    sonarMsgB = receive(sonarback, 1);
    sonarMsgD = receive(sonarder, 1);
    sonarMsgI = receive(sonarizq, 1);
    sonarMsgB2 = receive(sonarback2, 1);
    % Extraer la distancia a las paredes
    izqDist = sonarMsgI.Range_;
    derDist = sonarMsgD.Range_;  
    frontDist = sonarMsgF.Range_;
    backDist = sonarMsgB.Range_;
    backdist2 = sonarMsgB2.Range_;
    
    % Definimos la dist max a la q debe detectar pared
    distm = 4;
    % Activar los controladores en relacion a las paredes q hay
    if izqDist < distm 
        disp("Hay una pared a la izquierda a "+izqDist+" metros");
        
        paredes(1) = true;
    end
    if derDist < distm
        disp("Hay una pared a la derecha a "+derDist+" metros");
        
        paredes(2) = true;
    end
    if frontDist < distm
        disp("Hay una pared enfrente a "+frontDist+" metros");
       
        paredes(3) = true;
    end
    if backDist < distm && backdist2 < distm
        disp("Hay una pared detras a "+backDist+" metros");
       
        paredes(4) = true;
    end
end

function girarder()
    rosshutdown;
    %% INICIALIZACIÓN DE ROS
    % Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP
    %de la máquina donde se ejecuta Matlab). Si se está conectado a la misma red, la
    %variable ROS_IP no es necesario definirla.
    setenv('ROS_MASTER_URI','http://192.168.0.15:11311');
    setenv('ROS_IP','192.168.0.14') %Aqui poner la ip de windows
    rosinit() % Inicialización de ROS en la IP correspondiente
    %% DECLARACIÓN DE SUBSCRIBERS
    odom=rossubscriber('/robot0/odom'); % Subscripción a la odometría
    receive(odom);
    %% DECLARACIÓN DE PUBLISHERS
    pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
    %% GENERACIÓN DE MENSAJE
    msg=rosmessage(pub); %% Creamos un mensaje del tipo declarado en "pub (geometry_msgs/Twist)


    ori_inicial = odom.LatestMessage.Pose.Pose.Orientation;

    yaw_ini = quat2eul([ori_inicial.W ori_inicial.X ori_inicial.Y ori_inicial.Z]);
    yaw_ini = yaw_ini(1);
    
    
    posFinal = round(rad2deg(yaw_ini),0) -89 ;
    
    if (posFinal > 180) && (posFinal >0)
        posFinal =  - (180- (abs(180 - abs(-89)))) ;
    
    elseif (posFinal < -180) && (posFinal <0)
        posFinal =  (180- (abs(180 - abs(-89))));
        
    end
    
    yaw_fin= deg2rad(posFinal);
    
    arreglargiro = round(yaw_fin,1);
    
    
    if (arreglargiro > 1) && (arreglargiro < 2)
        yaw_fin = 1.5708;%90º
    
    elseif (arreglargiro < -1) && (arreglargiro > -2)
        yaw_fin = -1.5708;%-90º
        
    elseif (arreglargiro > 2) && (arreglargiro < 4)
        yaw_fin = 3.1416;
        
    elseif (arreglargiro < -2) && (arreglargiro > -4)
        yaw_fin = -3.1416;
        
    elseif (arreglargiro < 1) && (arreglargiro > -1)
        yaw_fin = 0;
    end
    
    errorAngulo=0.009;
    
    while (1)
       ori_inicial = odom.LatestMessage.Pose.Pose.Orientation;
       yaw_ac = quat2eul([ori_inicial.W ori_inicial.X ori_inicial.Y ori_inicial.Z]);
       yaw_ac = yaw_ac(1);
       
       msg.Linear.X = 0.0;
    
            msg.Angular.Z = -0.1;
    
       if (abs(yaw_ac - yaw_fin)<errorAngulo)
            msg.Angular.Z =0.0;
            msg.Linear.X = 0.0;
            send(pub,msg);
            break;
       end
       
       send (pub,msg);
    end
end

function girarizq()
    rosshutdown;
    %% INICIALIZACIÓN DE ROS
    % Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP
    %de la máquina donde se ejecuta Matlab). Si se está conectado a la misma red, la
    %variable ROS_IP no es necesario definirla.
    setenv('ROS_MASTER_URI','http://192.168.0.15:11311');
    setenv('ROS_IP','192.168.0.14') %Aqui poner la ip de windows
    rosinit() % Inicialización de ROS en la IP correspondiente
    %% DECLARACIÓN DE SUBSCRIBERS
    odom=rossubscriber('/robot0/odom'); % Subscripción a la odometría
    receive(odom);
    %% DECLARACIÓN DE PUBLISHERS
    pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
    %% GENERACIÓN DE MENSAJE
    msg=rosmessage(pub); %% Creamos un mensaje del tipo declarado en "pub (geometry_msgs/Twist)

ori_inicial = odom.LatestMessage.Pose.Pose.Orientation;
giro=90;
yaw_ini = quat2eul([ori_inicial.W ori_inicial.X ori_inicial.Y ori_inicial.Z]);
yaw_ini = yaw_ini(1);


posFinal = round(rad2deg(yaw_ini),0) + giro ;

if (posFinal > 180) && (posFinal >0)
    posFinal =  - (180- (abs(180 - abs(giro)))) ;

elseif (posFinal < -180) && (posFinal <0)
    posFinal =  (180- (abs(180 - abs(giro))));
    
end

yaw_fin= deg2rad(posFinal);

arreglargiro = round(yaw_fin,1);




errorAngulo=0.009;

while (1)
   ori_inicial = odom.LatestMessage.Pose.Pose.Orientation;
   yaw_ac = quat2eul([ori_inicial.W ori_inicial.X ori_inicial.Y ori_inicial.Z]);
   yaw_ac = yaw_ac(1);
   
   msg.Linear.X = 0.0;
   msg.Angular.Z = 0.1;


   if (abs(yaw_ac - arreglargiro)<errorAngulo)
        msg.Angular.Z =0.0;
        msg.Linear.X = 0.0;
        send(pub,msg);
        break;
   end
   
   send (pub,msg);

end
end

function avanzarAtras()
    girarder();
    girarder();
    avanzar(4.25);
    girarder();
    girarder();
end

function mapear()
    %% INICIALIZACIÓN DE ROS
    % Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP
    %de la máquina donde se ejecuta Matlab). Si se está conectado a la misma red, la
    %variable ROS_IP no es necesario definirla.
    setenv('ROS_MASTER_URI','http://192.168.0.15:11311');
    setenv('ROS_IP','192.168.0.14') %Aqui poner la ip de windows
    rosshutdown;
    rosinit() % Inicialización de ROS en la IP correspondiente
   
    


    %Empieza la función aquí
    mapa = zeros(25, 3); %Creamos matriz
    x = 0;
    y = 0;
    
    casillas = 1;
 
    movimientos = {};
    girarizq();
    avanzar(4.25);
    girarder();
    
    x = x - 1;
    mapa(casillas, 1) = casillas;
    mapa(casillas, 2) = x;
    mapa(casillas, 3) = y;
    casillas = casillas + 1; %Al visitar una nueva casilla añade 1 al contador de casillas nuevas
    movimientos{end + 1} = 1;
    while(casillas < 27)%solo 27
        disp(mapa);
        disp(movimientos);
        paredes = detectarParedes(); %Miramos que posibles caminos tiene disponibles
        caminosLibres = 0; 
        
        visitadaI = false;
        visitadaD = false;
        visitadaF = false;
        visitadaB = false;
        %miramos cuantos caminos posibles tiene
        for i = 1:4  
            if(paredes(i) == true)
                caminosLibres = caminosLibres + 1;
            end
        end
        %Comprobamos si alguno de los libres ha sido visitado
        for i = 1:4 
                if(i == 1) %izq
                    xaux = x -1;
                    yaux = y;
                    disp("si fuese hacia izquierda seria: "+xaux+" y "+yaux)
                    % Obtener el número de filas de la matriz
                    numFilas = size(mapa, 1);

                    % Recorrer cada fila y mostrar sus datos
                    for j = 1:numFilas
                        if(mapa(j, 2) == xaux && mapa(j, 3) == yaux)
                            igualX = true;
                             igualY = true;
                        else 
                            igualX = false;
                             igualY = false;
                       
                        end
                        if(igualX == true) && (igualY== true)
                            visitadaI = true;
                            disp("La izquierda ya ha sido visitada");
                        end
                    end
                end
                if(i == 2) %der
                   
                    xaux = x + 1;
                    yaux = y;
                    disp("si fuese hacia derecha: "+xaux+" y "+yaux)
                    % Obtener el número de filas de la matriz
                    numFilas = size(mapa, 1);

                    % Recorrer cada fila y mostrar sus datos
                    for h = 1:numFilas
                        if(mapa(h, 2) == xaux && mapa(h, 3) == yaux)
                            igualX = true;
                             igualY = true;
                        else 
                            igualX = false;
                             igualY = false;
                       
                        end
                        if(igualX == true) && (igualY== true)
                            visitadaD = true;
                            disp("La derecha ya ha sido visitada");
                        end
                    end
                end
                if(i == 3) %front
                   
                    xaux = x;
                    yaux = y + 1;
                    disp("si fuese hacia delante seria: "+xaux+" y "+yaux)
                    % Obtener el número de filas de la matriz
                    numFilas = size(mapa, 1);

                    % Recorrer cada fila y mostrar sus datos
                    for a = 1:numFilas
                        if(mapa(a, 2) == xaux && mapa(a, 3) == yaux)
                            igualX = true;
                             igualY = true;
                        else 
                            igualX = false;
                             igualY = false;
                       
                        end
                        if(igualX == true) && (igualY == true)
                            visitadaF = true;
                            disp("El frente ya ha sido visitado");
                        end
                    end
                end
                if(i == 4) %back
                 
                    xaux = x ;
                    yaux = y - 1;
                    disp("si fuese hacia atras seria: "+xaux+" y "+yaux)
                    % Obtener el número de filas de la matriz
                    numFilas = size(mapa, 1);

                    % Recorrer cada fila y mostrar sus datos
                    for e = 1:numFilas
                        if(mapa(e, 2) == xaux && mapa(e, 3) == yaux)
                            igualX = true;
                             igualY = true;
                        else 
                            igualX = false;
                             igualY = false;
                       
                        end
                        if(igualX == true) && (igualY == true)
                            visitadaB = true;
                            disp("La celda de detras ya ha sido visitada");
                        end
                    end
                end
            end
   
        
        %Acabamos las comprobaciones
        %La prioridad va en base a:
        %Primero de todo una celda q no haya sido visitada
        % Izquierda Libre > Frente Libre > Derecha Libre > Detras Libre 
        if(~visitadaI)||(~visitadaD)||(~visitadaF)||(~visitadaB) %Si existen caminos a los que ir que no han sido visitados nos movemos
            disp("---------------------------------------------------------------")
            disp("caminos libres: "+caminosLibres);
            disp("paredes: "+paredes);
            disp("VisitadaI: "+visitadaI);
            disp("VisitadaD: "+visitadaD);
            disp("VisitadaF: "+visitadaF);
            disp("VisitadaB: "+visitadaB);
            disp("X es: "+x);
            disp("Y es: "+y);
            disp(movimientos);
            
             if(~visitadaI) && (~paredes(1)) %si izq no esta visitada y esta libre
                disp("La izquierda no esta visitada y esta libre");
                disp("Gira a la izquierda y avanza, vuelve a girar a la derecha");
                girarizq();
                avanzar(4.25);
                girarder();
               
                x = x - 1;
                mapa(casillas, 1) = casillas;
                mapa(casillas, 2) = x;
                mapa(casillas, 3) = y;
                casillas = casillas + 1; %Al visitar una nueva casilla añade 1 al contador de casillas nuevas
                movimientos{end + 1} = 1;
                %Tengo que meter los datos en la matriz ya que es nueva
            elseif(~visitadaF) && (~paredes(3)) %si front no esta visitada y esta libre
                disp("El frente no esta visitado y esta libre, por lo que avanza");
                avanzar(4.25);
                %Tengo que meter los datos en la matriz ya que es nueva
                y = y + 1;
                
                mapa(casillas, 1) = casillas;
                mapa(casillas, 2) = x;
                mapa(casillas, 3) = y;
                casillas = casillas + 1; %Al visitar una nueva casilla añade 1 al contador de casillas nuevas
                movimientos{end + 1} = 3;
            elseif(~visitadaD) && (~paredes(2)) %si der no esta visitada y esta libre
                disp("Derecha libre, giro de derechas avanza y giro de izquierdas ");
                girarder();
                avanzar(4.25);
                girarizq();
                x = x + 1;
               
                mapa(casillas, 1) = casillas;
                mapa(casillas, 2) = x;
                mapa(casillas, 3) = y;
                casillas = casillas + 1; %Al visitar una nueva casilla añade 1 al contador de casillas nuevas
                %Tengo que meter los datos en la matriz ya que es nueva
                movimientos{end + 1} = 2;
            elseif(~visitadaB) && (~paredes(4)) %si back no esta visitada y esta libre
                disp("Por detras esta libre luego avanza Atrás")
                avanzarAtras();
                y = y - 1;
              
                mapa(casillas, 1) = casillas;
                mapa(casillas, 2) = x;
                mapa(casillas, 3) = y;
                casillas = casillas + 1; %Al visitar una nueva casilla añade 1 al contador de casillas nuevas
                %Tengo que meter los datos en la matriz ya que es nueva
                movimientos{end + 1} = 4;
            
            else %si todos los caminos han sido visitados
            disp("TODAS LAS CELDAS A SU ALREDEDOR HAN SIDO VISITADAS");
            if(movimientos{end} == 1)
                disp("El movimiento previo fue el de desplazarse a la izquierda");
                x = x + 1;
                girarder();
                avanzar(4.25);
                girarizq();  
                movimientos = movimientos(1:end-1);
              elseif(movimientos{end} == 3)
                disp("El movimiento previo fue el de desplazarse frente");
                y = y - 1;
                avanzarAtras();
                movimientos = movimientos(1:end-1);
              elseif(movimientos{end} == 2)
                disp("El movimiento previo fue el de desplazarse derecha"); 
                 x = x - 1;
                girarizq();
                avanzar(4.25);
                girarder();
                movimientos = movimientos(1:end-1);
              elseif(movimientos{end} == 4)
                disp("El movimiento previo fue el de desplazarse atras");
                avanzar(4.25);
                y = y + 1;
                movimientos = movimientos(1:end-1);
             end
        end
    end
    end
end


























