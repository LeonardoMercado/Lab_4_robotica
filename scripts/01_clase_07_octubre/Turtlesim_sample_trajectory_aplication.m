%UNIVERSIDAD NACIONAL DE COLOMBIA
%DEPARTAMENTO DE ING. MECANICA Y MECATRONICA
%ROBOTICA 2020-II
%EJEMPLO DE MANEJO DE ROS + MATLAB
%APLICACION SIMPLE - PARAMETRIZACION DE TRAYECTORIAS (EN EL PLANO)

%AUTOR: JURGEN KREJCI MUÑOZ
%CREACION: 4 DE OCTUBRE, 2020
%MAIL: jhkrejcim@unal.edu.co

%Antes de ejecutar asegurarse de tener en ejecución
%el nodo tutlesim en ROS.
%Usar:  'rosrun turtlesim turtlesim_node'
%NOTA: Asegurarse de que en el nodo se la simulación
%solo se encuentre una tortuga con el nombre /turtle1

%% Clear Workspace 
clear;
clc;

%% Parametrización de curva

% Based on http://people.math.harvard.edu/~knill/teaching/summer2011/handouts/23-curves.pdf

syms t x(t) y(t);

x_rad = 4;
y_rad = 1;

x(t) = x_rad*cos(t);
y(t) = y_rad*sin(t);

t = [0:0.1:2*pi];
x_out = double(subs(x));
y_out = double(subs(y));

plot(x_out,y_out,'--');
grid on;
title('Parametrización elipse');
xlabel('x');
ylabel('y');

%% Calculo de velocidades

% Based on https://www.math24.net/curvature-radius/

% Calculo radio de curvatura

x_d = diff(x); % Derivadas
y_d = diff(y);

x_dd = diff(x_d);
y_dd = diff(y_d);

K = ((x_d*y_dd)-(y_d*x_dd))/((x_d^2 + y_d^2)^(3/2)); %Curvatura
R = 1/K; %Radio de curvatura
 
%Velocidad lineal
V = (x_d^2 + y_d^2)^(1/2);

%Velocidad angular
W = V/R;

V_out = double(subs(V));
W_out = double(subs(W));

subplot(2,1,1);
plot(t,V_out,'r--');
grid on;
title('Velocidad Lineal');
xlabel('t [s]');
ylabel('V [m/s]');

subplot(2,1,2);
plot(t,W_out,'b--');
grid on;
title('Velocidad Angular');
xlabel('t [s]');
ylabel('V [rad/s]');

%% ROS intialization and turtle setup
disp('ROS Initilization');
rosinit;
disp('ROS Ready');

%Service for teleportation to initial position
teleport_turtle_client = rossvcclient('/turtle1/teleport_absolute');
teleport_msg = rosmessage(teleport_turtle_client);
teleport_msg.X = 10.0;
teleport_msg.Y = 5.0;
teleport_msg.Theta = pi/2;
call(teleport_turtle_client,teleport_msg,'Timeout',3);

%Background setup
rosparam("set", "/turtlesim/background_r", 0);
rosparam("set", "/turtlesim/background_g", 255);
rosparam("set", "/turtlesim/background_b", 255);

%Turtle Line setup
turtle_client = rossvcclient('/turtle1/set_pen');
request_msg = rosmessage(turtle_client);
request_msg.R = 255;
request_msg.G = 0;
request_msg.B = 255;
request_msg.Width = 4;
call(turtle_client,request_msg,'Timeout',3);

%Clearing and displaying updated params
clear_client = rossvcclient('/clear');
clear_msg = rosmessage(clear_client);
call(clear_client,clear_msg,'Timeout',3);

%Topic for velocity 
commander = rospublisher('/turtle1/cmd_vel', 'geometry_msgs/Twist');
pause(1);


%% Trayectory execution using timer
tiempo = 2*pi;
periodo = 0.1;
ciclos = round(tiempo/periodo);

global tm;
global elapsed;
global pub_flag;

elapsed = 0;
pub_flag = 0;


tm = timer('StartDelay', 2, 'Period', periodo, 'TasksToExecute', ciclos, ...
          'ExecutionMode', 'fixedRate');

tm.StartFcn = {@limit_callback_fcn};
tm.StopFcn = {@limit_callback_fcn};
tm.TimerFcn = {@timer_callback_fcn};

disp('Starting timer...');
tm.start;

counter = 1;

while counter < 63
    
    if pub_flag == 1
        
       V_lin = V_out(counter);
       W_ang = W_out(counter);
       speed_command = rosmessage(commander);
       speed_command.Linear.X = V_lin;
       speed_command.Angular.Z = W_ang;
       send(commander,speed_command);
       pub_flag = 0;
       counter = counter+1;
    end
    
end

rosshutdown;


%Funciones (Timer)
      
function limit_callback_fcn(~, event)

txt = 'event occurred at ';

event_type = event.Type;
event_time = datestr(event.Data.time);

msg = [event_type txt event_time];
disp(msg)

end

function timer_callback_fcn(~,event)

global tm;
global elapsed;
global pub_flag;

pub_flag = 1;
if tm.TasksExecuted > 1
    elapsed = elapsed + tm.InstantPeriod;
end
fprintf('Elapsed time \n');
disp(elapsed);

end