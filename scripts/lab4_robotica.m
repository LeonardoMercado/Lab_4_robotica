%% Laboratorio #4 robótica 2020-2
%
% Leonardo Fabio Mercado Benítez
%
% C.C: 1.016.050.737
%
% Código: 25481090
%
%
%% Modelo del robot phanton X pincher:
clc;
clear;
close all;

syms Q1 Q2 Q3 Q4

l1 = 0.135875;
l2 = 0.107;
l3 = 0.107;
l4 = 0.091;


L(1) = Link('revolute','alpha', 0,    'a',0,   'd',l1,  'offset', 0,   'modified', 'qlim',[-2*pi 2*pi]);
L(2) = Link('revolute','alpha', pi/2, 'a',0,   'd',0,   'offset', pi/2, 'modified', 'qlim',[-2*pi 2*pi]);
L(3) = Link('revolute','alpha', 0,    'a',l2,  'd',0,   'offset', 0, 'modified', 'qlim',[-2*pi 2*pi]);
L(4) = Link('revolute','alpha', 0,    'a',l3,  'd',0,   'offset', 0,   'modified', 'qlim',[-2*pi 2*pi]);
        

robot = SerialLink(L,'name','Phantom_x');
robot.tool = [0 0 1 l4;
              1 0 0 0;
              0 1 0 0;
              0 0 0 1];
maximo = 0.800;
robot.plot([0 0 0 0],'workspace',[-maximo maximo -maximo maximo 0 maximo],'noa');
robot.teach;        

