clear; close all; clc;
%Load required packages
pkg load control;
pkg load signal;
%% Motor Constants
L = 0.5;   %H
R = 1;     %Ohm

J = 0.1;  %kg/m^2
b = 0.1;    %Ns/m
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
C = [1 0 0];
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
t = linspace(0,50, 1000);
x0 = zeros(1,8);

%Simulation parameters
x0(1) = 5;      %Set terminal voltage 
gamma = 1;      %Set adapatation gain
enable = 0;     %Disable the MRAS system

[y, tOut] = step(motor);

%%Auto-tune the PID controller for the motor position
k = max(y);

A0 = max(y)*max(tOut)-trapz(tOut,y);
[M, I] = min(abs(t-(A0/k)));
A1 = trapz(tOut(1:I), y(1:I));
T = (exp(1)*A1)/k;
L = (A0/k)-T;
alpha = L*((0.63*k)/T);

%Pregenerate PID values
P = 0;
I = 0;
D = 0;
%PID Control
P = 1.2/alpha;
I = 2*L;
D = L/2;

%Create system to plot
C = pid(P, P/I, P*D);
%T = feedback(ss2tf(motor)*tf(C), 1);
step(ss2tf(motor))