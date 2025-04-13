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
%step(motor)
%Implement Model Reference Adapative System for Inertia

function px = g(x, t, gamma, enable)
    J0 = 0.005; %Reference model gain
                %What the MRAS starts with as an estimate

    % Motor Constants
    L = 0.5;   %H
    R = 1;     %Ohm
    %J is the actual inertia of the motor 
    J = 0.01;  %kg/m^2
    B = 0.1;    %Ns/m
    Ke = 0.01; %V/rad/sec
    Kt = 0.01; %Nm/Amp
    K = Ke;    %Since kt = Ke only need one variable

    %Constant input voltage to the motor
    uc = 5;


    %x vector map:
    %x(1) = theta_dot
    %x(2) = current
    %x(3) = input voltage
    %x(4) = Theta_dot_MRAS
    %x(5) = current_MRAS
    %x(6) = d(theta)/dt -> Control update signal for speed
    %x(7) = d(theta_2)/dt -> Control update signal for current
    %Output vector
    px = zeros(1,7);
    % If enable is 0 do not run the MRAS
    % Just run the system in an open loop configuration
    if(enable == 0)
        %State 1, theta_dot
        px(4) = x(4)*(-B/J)+x(5)*(K/J);
        px(5) = x(4)*(-K/J)+x(5)*(-R/L)+uc*L;
        %As derived from the state space model,
        % y = theta_dot -> y is speed control
    endif
    % If enable is 1 run the MRAS 
    % This means updating the system to try and
    %determine teh current inertia of the system 
    if(enable == 1)
        %Use estimated inertia, J0
        %MRAS States
        px(4) = x(4)*(-B/J0)+x(5)*(K/J0);
        px(5) = x(4)*(-K/J0)+x(5)*(-R/L)+uc*L;

        %Update control signal based on theta
        %See example 5.1 pg. 187
        u = x(6)*uc;
        px(3) = u;
        %Now implement plant with the modified 
        %control signal
        px(1) = x(1)*(-B/J)+x(2)*(K/J);
        px(2) = x(1)*(-K/J)+x(2)*(-R/L)+u*L;
    
        %Update MIT Rule
        % e = y - ym 
        e = x(1) - x(4);
        %Update input for next loop
        %In this case ym is a roational speed
        px(6) = -gamma*e*x(4);
    endif
endfunction;

%Simulation
t = linspace(0,120, 10000);
x0 = zeros(1,7);
%Set an input voltage to make the system do something
%This is the same as a step input at t=0
%MIT Rule gamma
gamma = 5;

%% Simulation open loop motor
x = lsode(@(x,t) g(x,t,gamma, 0), x0, t);
%Plot results of open loop operation
figure()
plot(t,x(:,4))
title("Open Loop Step Response");
ylabel("theta_dot rads/sec");
xlabel('Time (Seconds)')
%%  Simulation MRAS motor
x = lsode(@(x,t) g(x,t,gamma, 1), x0, t);
%Plot results of MRAS 
figure()
plot(t,x(:,1), t, x(:,4))
title("MRAS System Step Response");
legend("y", "ym");
ylabel("theta_dot rads/sec");
xlabel('Time (Seconds)')



pause(0.25);