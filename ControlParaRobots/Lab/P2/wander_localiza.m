%Cargar el mapa
load mi_mapaSlamConRuido.mat map
show(map);
%Crear el objeto VFH y ajustar sus propiedades
VFH=controllerVFH;
VFH.NumAngularSectors=180;
VFH.DistanceLimits=[0.05 2];
VFH.RobotRadius=0.1;
VFH.SafetyDistance=0.1;
VFH.MinTurningRadius=0.1;
VFH.TargetDirectionWeight=5;
VFH.CurrentDirectionWeight=2;
VFH.PreviousDirectionWeight=2;
VFH.HistogramThresholds=[3 10];
VFH.UseLidarScan=true; %para permitir utilizar la notación del scan
targetDir = 0;

%Inicializar el localizador AMCL (práctica 1)

%% Setup the laser sensor model and amigobot motion model
% AmigoBot can be modeled as a differential drive robot and its motion can be estimated
% using odometry data. The |Noise| property defines the uncertainty in robot's rotational
% and linear motion. Increasing the |odometryModel.Noise| property will allow more spread
% when propagating particles using odometry measurements.
% Please refer to <docid:robotics_ref.bu359h6-1 |robotics.OdometryMotionModel|> for property details.
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

% The sensor on amigobot is a simulated/real range finder converted from laser readings.
% The likelihood field method is used to compute the probability of perceiving a set of
% measurements by comparing the end points of the range finder measurements to the occupancy map.
% If the end points match the occupied points in occupancy map, the probability of perceiving
% such measurements is high. The sensor model should be tuned to match the actual sensor property
% to achieve better test results. The property |SensorLimits| defines the minimum and maximum
% range of sensor readings. The property |Map| defines the occupancy map used for computing
% likelihood field.Please refer to <docid:nav_ref.bu31hrp-1 |likelihoodFieldSensorModel|>
% for property details.
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 8];
rangeFinderModel.Map = map;

% Set |rangeFinderModel.SensorPose| to the coordinate transform of the fixed laser sensor
% with respect to the robot base. This is used to transform the laser readings from laser frame
% to the base frame of AmigoBot. Please refer to
% <docid:robotics_examples.example-ROSTransformationTreeExample docid:robotics_examples.example-ROSTransformationTreeExample>
% for details on coordinate transformations.
%
% Note that currently |SensorModel| is only compatible with sensors that are fixed
% on the robot's frame, which means the sensor transform is constant.

% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
%Obtener transformada entre los frames del robot y del sensor_laser
waitForTransform(tftree,'/robot0','/robot0_laser_1');
sensorTransform = getTransform(tftree,'/robot0', '/robot0_laser_1');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's +X, +Y direction in meters
% and rotation angle along base_link's +Z axis in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

%% Interface for receiving sensor measurements from AmigoBot and sending velocity commands to AmigoBot.
% Create ROS subscribers for retrieving sensor and odometry measurements from TurtleBot.
% sub_laser = rossubscriber('scan'); sub_odom = rossubscriber('odom');

% Hecho en ini_simulator.m o ini_amigobot.m
%%
% Create ROS publisher for sending out velocity commands to AmigoBot. TurtleBot subscribes to
% |'/mobile_base/commands/velocity'| for velocity commands. [velPub,velMsg]
% = ... rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');
% Hecho en ini_simulator.m o ini_amigobot.m
% Se usa pub_vel y msg_vel
%% Initialize AMCL object
% Instantiate an AMCL object |amcl|. See <docid:nav_ref.bu31hfz-1 monteCarloLocalization>
% for more information on the class.
amcl = monteCarloLocalization;
amcl.UseLidarScan = true;

% Assign the |MotionModel| and |SensorModel| properties in the |amcl| object.
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

% The particle filter only updates the particles when the robot's movement exceeds the
% |UpdateThresholds|, which defines minimum displacement in [x, y, yaw] to trigger filter update.
% This prevents too frequent updates due to sensor noise. Particle resampling happens after the
% |amcl.ResamplingInterval| filter updates.
% Using larger numbers leads to slower particle depletion at the price of slower particle convergence
% as well.
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

%% Configure AMCL object for localization with initial pose estimate.
% |amcl.ParticleLimits| defines the lower and upper bound on the number of particles  that will be
% generated during the resampling process. Allowing more particles to be generated may improve
% the chance of converging to the true robot pose, but has an impact on computation speed
% and particles may take longer time or even fail to converge. Please refer to the 'KL-D Sampling'
% section in [1] for computing a reasonable bound value on the number of particles. Note that global
% localization may need significantly more particles compared to localization with an initial pose
% estimate. If the robot knows its initial pose with some uncertainty, such additional information
% can help AMCL localize robots faster with a less number of particles, i.e. you can use a smaller
% value of upper bound in |amcl.ParticleLimits|.
%
% Now set |amcl.GlobalLocalization| to false and provide an estimated initial pose to AMCL.
% By doing so, AMCL holds the initial belief that robot's true pose follows a Gaussian distribution
% with a mean equal to |amcl.InitialPose| and a covariance matrix equal to
% |amcl.InitialCovariance|. Initial pose estimate should be obtained according to your setup.
% This example helper retrieves the robot's current true pose from Gazebo.
%
% Please refer to section Configure AMCL object for global localization for an example on using
% global localization.
amcl.ParticleLimits = [500 50000];           % Minimum and maximum number of particles
amcl.GlobalLocalization = true;      % global = true      local=false
amcl.InitialPose = [0 0 0];              % Initial pose of vehicle
amcl.InitialCovariance = diag([0.5 0.5 0.5]); % Covariance of initial pose
%diag([0.5 0.5 0.5])*...

%% Setup helper for visualization and driving AmigoBot.
% Setup ExampleHelperAMCLVisualization to plot the map and update robot's estimated pose, particles,
% and laser scan readings on the map.
visualizationHelper = ExampleHelperAMCLVisualization(map);

%% Robot Motion
% Robot motion is essential for the AMCL algorithm. In this example, we drive Amigobot randomly
% using the "rosrun teleop_twist_keyboard teleop_twist_keyboard.py" class, which drives the robot
% inside the environment while avoiding obstacles using the
% |<docid:robotics_ref.buv7g7y robotics.VectorFieldHistogram>| class.
%wanderHelper = ...
%    ExampleHelperAMCLWanderer(sub_laser, sensorTransform, pub_vel, msg_vel);

%% Localization procedure
% The AMCL algorithm is updated with odometry and sensor readings at each time step when the robot
% is moving around. Please allow a few seconds before particles are initialized and plotted
% in the figure. In this example we will run |numUpdates| AMCL updates. If the robot doesn't converge
% to the correct robot pose, consider using a larger |numUpdates|.
i=1;
%Rellenamos los campos por defecto de la velocidad del robot, para que la lineal sea siempre 0.1 m/s
msg_vel.Linear.X = 0.1;
msg_vel.Linear.Y = 0;
msg_vel.Linear.Z = 0;
msg_vel.Angular.X = 0;
msg_vel.Angular.Y = 0;
msg_vel.Angular.Z = 0;
send(pub_vel,msg_vel);

%Bucle de control infinito
while(1)

    %Leer y dibujar los datos del láser en la figura ‘fig_laser’
    msg_laser = sub_laser.LatestMessage;
    figure(fig_laser);
    plot(msg_laser, 'MaximumRange', 8);
    scan = lidarScan(msg_laser);
    %Leer la odometría
    sub_odom = rossubscriber('/robot0/odom', 'nav_msgs/Odometry');
    %Obtener la posición pose=[x,y,yaw] a partir de la odometría anterior
    odomQuat = [sub_odom.Pose.Pose.Orientation.W, sub_odom.Pose.Pose.Orientation.X, ...
        sub_odom.Pose.Pose.Orientation.Y, sub_odom.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [sub_odom.Pose.Pose.Position.X, sub_odom.Pose.Pose.Position.Y odomRotation(1)];
    %Ejecutar amcl para obtener la posición estimada estimatedPose y la covarianza estimatedCovariance
    %(mostrar la última por pantalla para facilitar la búsqueda de un umbral)
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);
    disp(estimatedCovariance);


    %Si la covarianza está por debajo de un umbral, el robot está localizado y
    %finaliza el programa
    if (estimatedCovariance(1,1)<umbralx && estimatedCovariance(2,2)<umbraly && estimatedCovariance(3,3)<umbralyaw)
        disp('Robot Localizado');
        break;
    end
    %Dibujar los resultados del localizador con el visualizationHelper

    %Llamar al objeto VFH para obtener la dirección a seguir por el robot para
    %evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
    %en la figura ‘fig_vfh’

    %Rellenar el campo de la velocidad angular del mensaje de velocidad con un
    %valor proporcional a la dirección anterior (K=0.1)

    %Publicar el mensaje de velocidad
    %Esperar al siguiente periodo de muestreo
end
