% Octave
% Created April 14th 2025
% Author: Drew Currie
% Created as the final project for MSU EELE-592
% This project will work to model the current load
% on a DC motor through the change in the moment of 
% inertia. 
%
%This file creates all the dc motor objects
%and calls the various functions on that object 
%to run the simulation

clear; close all; clc;
%Load required packages
pkg load control;

%% Motor Constants
L = 0.5;   %H
R = 1;     %Ohm

J = 0.01;  %kg/m^2
b = 0.1    %Ns/m
Ke = 0.01; %V/rad/sec
Kt = 0.01; %Nm/Amp
K = Ke;    %Since kt = Ke only need one variable



%% Motor Equations
A = [0,1,0; 0,-b/J,K/J; 0,-K/L, -R/L];
B = [0; 0; 1/L];
% x = [x1;x2]
%x1 = theta_dot x2 = i
%d/dt(x) = Ax+Bv
%Set C matrix for speed output
C = [0 1 0];
%y = C*x
%Step motor just for reference comapred to lsode solution
motor = ss(A,B,C,0);
%% Model reference system
%Implement Model Reference Adapative System for Inertia
J0 = 0.01;      %Set desired inertia behavior
A0 = [0, 1, 0; 0,-b/J0,K/J0; 0,-K/L,-R/L;];
model = ss(A0, B, C, 0);
%% Simulation
%Set t and initial conditions
t = linspace(0,300, 10000);
x0 = zeros(1,8);

%Simulation parameters
x0(1) = 5;      %Set terminal voltage 
gamma = 1;      %Set adapatation gain
enable = 0;     %Disable the MRAS system

%% Simulation open loop motor
x = lsode(@(x,t) dcmotor_speedcontrol(x, t, enable, gamma, motor, model), x0, t);
%Plot results of open loop operation
figure()
plot(t,x(:,3))
hold on;
title("Open Loop Step Response");
ylabel("theta_dot rads/sec");
xlabel('Time (Seconds)')

%Simulation parameters
enable = 1;     %Enable the MRAS system 

%%  Simulation MRAS motor
x = lsode(@(x,t) dcmotor_speedcontrol(x, t, enable, gamma, motor, model), x0, t);
%Plot results of MRAS 
plot(t,x(:,3), t, x(:,6))
hold off;
title("MRAS System Step Response");
legend("open-loop","y", "ym");
ylabel("theta_dot rads/sec");
xlabel('Time (Seconds)')

%% Simulation of positional control
enable = 0;
%Move to 360 degrees 
x0(1) = 360;
x = lsode(@(x,t) dcmotor_positioncontrol(x, t, enable, gamma, motor, model), x0, t);

figure()
plot(t,x(:,3))
%hold on;
title("Open Loop Response, target = 2pi rads");
ylabel("theta radians");
xlabel('Time (Seconds)')

