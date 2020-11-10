%UNIVERSIDAD NACIONAL DE COLOMBIA
%DEPARTAMENTO DE ING. MECANICA Y MECATRONICA
%ROBOTICA 2020-II
%EJEMPLO DE MANEJO DE ROS + MATLAB
%COMANDOS BASICOS DE ROS

%AUTOR: JURGEN KREJCI MUÑOZ
%CREACION: 4 DE OCTUBRE, 2020
%MAIL: jhkrejcim@unal.edu.co

%% Clear Workspace and initialize ROS Node

clear;
clc;
rosinit;

%After ROS initialization, on terminal run the turtlesim node
%Use: 'rosrun turtlesim turtlesim_node'

%% Check active topics and info 
disp('Active topics:')
rostopic list;
fprintf('\n');
disp('Topic info for /turtle1/cmd_vel:');
rostopic info /turtle1/pose;
fprintf('\n');

%% Creating topic subscriber
pose = rossubscriber('/turtle1/pose');
pause(1);
fprintf('\n pose,');
disp(pose);

%% Message retriving
actual_pose = receive(pose,3);
disp(actual_pose);
%Esto es para un sólo mensaje.
%meter en un ciclo while para recibir varios mensajes.

%% Creating topic publisher
disp('Topic info for /turtle1/cmd_vel:')
rostopic info /turtle1/cmd_vel;

commander = rospublisher('/turtle1/cmd_vel', 'geometry_msgs/Twist');
pause(1);
fprintf('\n commander,');
disp(commander);%% Publishing Msgs 
speed_command = rosmessage(commander);

speed_command.Linear.X = 1;
speed_command.Angular.Z = 1;

send(commander,speed_command);
pause(2);

%% Checking available services and info

fprintf('\nAvailable services: \n');
rosservice list;
fprintf('\nService info for /turtle1/set_pen: \n');
rosservice info /turtle1/set_pen;

%% Creating service client and calling service

turtle_client = rossvcclient('/turtle1/set_pen');
fprintf('\nClient: \n');
disp(turtle_client);
fprintf('\nRequest message: \n');
request_msg = rosmessage(turtle_client);
disp(request_msg);

request_msg.R = 0;
request_msg.G = 255;
request_msg.B = 255;
request_msg.Width = 4;

fprintf('\nMessage details: \n');
request_msg.showdetails;

response = call(turtle_client,request_msg,'Timeout',3);
fprintf('\nMessage call: \n');
disp(response);

%% Parameter server values

fprintf('\nROS parameter list: \n');
rosparam list;

fprintf('\nBackground blue: \n');
background_b = rosparam("get","/turtlesim/background_b");
disp(background_b);

fprintf('\nChanging background color: \n');
rosparam("set", "/turtlesim/background_r", 125);
rosparam("set", "/turtlesim/background_g", 125);
rosparam("set", "/turtlesim/background_b", 0);

background_r = rosparam("get","/turtlesim/background_r");
disp(background_r);

% Call for /clear service to update background color

clear_client = rossvcclient('/clear');
clear_msg = rosmessage(clear_client);
response = call(clear_client,clear_msg,'Timeout',3);

%% Shutting down ROS

rosshutdown;

