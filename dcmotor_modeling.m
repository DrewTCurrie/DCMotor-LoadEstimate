% Octave
% Created April 14th 2025
% Author: Drew Currie
% Created as the final project for MSU EELE-592
% This project will work to model the current load
% on a DC motor through the change in the moment of 
% inertia. 

%This file creates all the dc motor objects
%and calls the various functions on that object 
%to run the simulation


clear; close all; clc;
%Load required packages
pkg load control;


% Motor Constants

L = 0.5;   %H
R = 1;     %Ohm

J = 0.01;  %kg/m^2
B = 0.1    %Ns/m
Ke = 0.01; %V/rad/sec
Kt = 0.01; %Nm/Amp
K = Ke;    %Since kt = Ke only need one variable

% Motor Equations
A = [-B/J,K/J; -K/L, -R/L];
B = [0; 1/L];
% x = [x1;x2]
%x1 = theta_dot x2 = i
%d/dt(x) = Ax+Bv
C = [1 0];
%y = C*x
%Step motor just for reference comapred to lsode solution
motor = ss(A,B,C,0);
%step(motor)
%Implement Model Reference Adapative System for Inertia




%Simulation
t = linspace(0,300, 10000);
x0 = zeros(1,7);
%Set an input voltage to make the system do something
%This is the same as a step input at t=0
%MIT Rule gamma
gamma = 1;

%% Simulation open loop motor
x = lsode(@(x,t) g(x,t,gamma, 0), x0, t);
%Plot results of open loop operation
figure()
plot(t,x(:,4))
hold on;
title("Open Loop Step Response");
ylabel("theta_dot rads/sec");
xlabel('Time (Seconds)')
%%  Simulation MRAS motor
x = lsode(@(x,t) g(x,t,gamma, 1), x0, t);
%Plot results of MRAS 
plot(t,x(:,1), t, x(:,4))
title("MRAS System Step Response");
legend("open-loop","y", "ym");
ylabel("theta_dot rads/sec");
xlabel('Time (Seconds)')



pause(0.25);