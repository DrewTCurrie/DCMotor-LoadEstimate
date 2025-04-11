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

motor = ss(A,B,C,0);
step(motor)

pause(0.25);
%Implement Model Reference Adapative System for Inertia

function px = g(t, x, gamma){
    k0 = 0.005; %Reference model gain
                %What the MRAS starts with as an estimate
    k = 0.01;   %Actual system gain. Real inertia of the motor

    %Output vector
    px = zeros(2,1);

    %State 1 
    px(1) = 

}