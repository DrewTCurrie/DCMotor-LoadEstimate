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

J = 0.5;  %kg/m^2
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
J0 = 0.1;      %Set desired inertia behavior
A0 = [0, 1, 0; 0,-b/J0,K/J0; 0,-K/L,-R/L;];
model = ss(A0, B, C, 0);
%% Simulation
%Set t and initial conditions
t = linspace(0,50, 1000);
x0 = zeros(1,9);

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

%% Calculate estimated inertia
% The speed control produces cleaner results
% So better to caclulate this now instead of
% after the position test

adapatation_gain_estimate = mean(diff(x(:, 9)));
Je = 1/((adapatation_gain_estimate/J0)/x(1));
je_error = (Je - J)/J * 100;

%% Simulation of positional control
x0(1) = 1;      %Set terminal voltage 
enable = 0;
%Move to 360 degrees 
x0(1) = 2*pi;
x = lsode(@(x,t) dcmotor_positioncontrol(x, t, enable, gamma, motor, model), x0, t);

figure()
plot(t,x(:,2))
%hold on;
title("Open Loop Response, target = 2pi rads");
ylabel("theta radians");
xlabel('Time (Seconds)')

%Update simulation parameters
x0(1) = 1;      %Set terminal voltage 
gamma = 5;      %Set adapatation gain
enable = 1;     %Disable the MRAS system
x = lsode(@(x,t) dcmotor_positioncontrol(x, t, enable, gamma, motor, model), x0, t);

figure()
plot(t,x(:,2), t, x(:,5))
title("MRAS Response, target = 2pi rads");
ylabel("theta radians");
xlabel('Time (Seconds)');
legend("y", "ym");


%Get PID parameters
pid_values = dcmotor_PIDautotune(motor);

%% MRAS Step response with PID 
%Create transfer function of motor
s = tf('s');
tf_motor = K/((J0*s+b)*(L*s+R)+K^2);
C = pid(pid_values(1), pid_values(2), pid_values(3));
T = feedback(tf(C)*tf_motor, 1);
[theta, tOut] = step(T);
%Plot step response of MRAS system
figure()
plot(tOut, theta);
title("MRAS system step response");
ylabel("theta radians");
xlabel('Time (Seconds)');
legend("theta");

%% Plant Step response with PID
%Create transfer function of motor
s = tf('s');

%% Estimated motor paramters
A = [0,1,0; 0,-b/Je,K/Je; 0,-K/L, -R/L];
B = [0; 0; 1/L];
% x = [x1;x2]
%x1 = theta_dot x2 = i
%d/dt(x) = Ax+Bv
%Set C matrix for speed output
C = [1 0 0];
%y = C*x
%Step motor just for reference comapred to lsode solution
motor = ss(A,B,C,0);
%Use estimated inertia to tune this version
pid_values = dcmotor_PIDautotune(motor);

%Still need the actual inertia for the plant
tf_motor = K/((J*s+b)*(L*s+R)+K^2);
C = pid(pid_values(1), pid_values(2), pid_values(3));
T = feedback(tf(C)*tf_motor, 1);
[theta, tOut] = step(T);
%Plot step response of MRAS system
figure()
plot(tOut, theta);
title("Motor system step response uisng PID and estimated inertia");
ylabel("theta radians");
xlabel('Time (Seconds)');
legend("theta");
fprintf("Calculated inertia: %f. Acutal inertia: %f. Percent error: %f.\n\r", Je, J, je_error)
pause(0.25);