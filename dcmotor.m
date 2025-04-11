% Octave
% Created April 10th 2025
% Author: Drew Currie
% Created as the final project for MSU EELE-592
% This project will work to model the current load
% on a DC motor through the change in the moment of 
% inertia. 

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
step(motor)
%Implement Model Reference Adapative System for Inertia

function px = g(x, t, gamma)
    k0 = 0.005; %Reference model gain
                %What the MRAS starts with as an estimate
    k = 0.01;   %Actual system gain. Real inertia of the motor

    % Motor Constants
    L = 0.5;   %H
    R = 1;     %Ohm

    J = 0.01;  %kg/m^2
    B = 0.1    %Ns/m
    Ke = 0.01; %V/rad/sec
    Kt = 0.01; %Nm/Amp
    K = Ke;    %Since kt = Ke only need one variable


    %x vector map:
    %x(1) = theta_dot
    %x(2) = current
    %x(3) = input voltage
    %Output vector
    px = zeros(3,1);

    %State 1, theta_dot
    px(1) = x(1)*(-B/J)+x(2)*(K/J);
    px(2) = x(1)*(-K/J)+x(2)*(-R/L)+x(3)*L;
    %Constant input voltage
    px(3) = x(3);
    %As derived from the state space model,
    % y = theta_dot -> y is speed control
endfunction;

%Simulation
t = linspace(0,3, 1000);
x0 = zeros(1,3);
%Set an input voltage to make the system do something
%This is the same as a step input at t=0
x0(3) = 1;
%MIT Rule gamma
gamma = 0.01;
x = lsode(@(x,t) g(x,t,gamma), x0, t);

pause(0.25);