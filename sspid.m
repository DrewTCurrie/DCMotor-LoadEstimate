% Octave
% Created April 15th 2025
% Author: Drew Currie
% Created as the final project for MSU EELE-592
% This project will work to model the current load
% on a DC motor through the change in the moment of 
% inertia. This function provides a state space PID 
% control for the motor. The PID values should be 
% calculated seperately and provided as an input
% to this function. 
%
% Call this function in a function being called by lsode
% or similar ODE solver
%
% System States
% x(1) = Reference Signal    -> System Input
% x(2) = Derivative State    -> Controller interal state
% X(3) = Proportional State  -> Controller internal state
% x(4) = Integral State      -> Controller internal state
% x(5) = y                   -> Plant feedback
% x(6) = u                   -> Controller output
%
% Additional required input parameters
% PID                        -> PID gains. Order of PID
%

function px = sspid(x, t, PID);
    %State space matrices are constant regardless of PID
    %gains or plant
    Ae = [0, 0, 0; 1, 0, 0; 0, 1, 0];
    Be = [1, 0, 0];
    Ce = [0, 1, 0];
    %B1, B2 and b0 are observer gains
    B1 = 0.09;
    B2 = 0.6;
    b0 = 0.0001;
    L0 = [B1, B2, 1]'/b0;
    %Augemented A matrix
    A = Ae-L0*Ce;
    %Preallocate vector for returning values
    px = zeros(1,6);

    %Calculate states
    px(2) = PID(2)*(1-x(4))+PID(3)*(1-x(2))+PID(1)*(1-x(3))+B1*x(3)+B1*x(5);
    px(3) = B2*x(3)+B2*x(5);
    px(4) = x(3)+x(5);

    %Calculate output
    x(6) = [PID(3), PID(1), PID(2)]*(x(1)-[x(2);x(3);x(4)]);
endfunction;

