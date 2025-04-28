% Octave
% Created April 27th 2025
% Author: Drew Currie
% Created as the final project for MSU EELE-592
% This project will work to model the current load
% on a DC motor through the change in the moment of 
% inertia. This system works to provide a 
% Model Reference Adpative Control for position control 
% of the DC motor. 
%
% Call this function to get PID values for a DC motor
% DC motor should be input as a statespace model.
% This function returns the corresponding PID values
%
%
% Motor is expected to be a state space sys model
% The system works off a modle for a DC motor with 
% position as a state. 
% 
% Model is expected to be a state space sys model
% The A matrix is expected to be 3x3 
% The A matrix should contain the system dynamics
% The B matrix is expected to be 1x3
% The C matrix is expected to be 3x1



function pid = dcmotor_PIDautotune(motor)
    %Load required packages
    pkg load control;
    pkg load signal;

    [y, tOut] = step(motor);
    figure()
    plot(y, tOut);

    %%Auto-tune the PID controller for the motor position
    k = max(y);

    A0 = max(y)*max(tOut)-trapz(tOut,y);
    [M, I] = min(abs(tOut-(A0/k)));
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
    pid = [P, I, D];
endfunction;
