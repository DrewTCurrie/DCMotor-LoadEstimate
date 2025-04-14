% Octave
% Created April 10th 2025
% Author: Drew Currie
% Created as the final project for MSU EELE-592
% This project will work to model the current load
% on a DC motor through the change in the moment of 
% inertia. This system works to provide a 
%Model Reference Adpative Control for speed control 
%of the DC motor. 
%
%Call this function using an ODE solver such as lsode
%
% x(1) = input voltage  -> Terminal Voltage
% X(2) = theta_dot      -> Rotational speed
% x(3) = current        -> Winding current
% x(4) = Theta_dot_MRAS -> MRAS rotational speed
% x(5) = Current_MRAS   -> MRAS Widing Current
% x(6) = d(theta)/dt    -> Control update signal
%
%Motor is expected to be a state space sys model
%The A matrix is expected to be 2x2
%The B matrix is expected to be 1x2
%The C matrix is expected to be 2x1
%Model is expected to be a state space sys model
%The A matrix is expected to be 2x2 
%The A matrix should contain the desired behavior
%The B matrix is expected to be 1x2
%The C matrix is expected to be 2x1

function px = dcmotor_speedcontrol(x, t, enable, gamma, motor, model)

    %System input, terminal voltage for the motor
    uc = x(1);
    %Output vector
    px = zeros(1,6);
    % If enable is 0 do not run the MRAS
    % Just run the system in an open loop configuration
    if(enable == 0)
        %State 1, theta_dot
        px(2) = x(2)*motor.a(1,1)+x(3)*motor.a(1,2);
        px(3) = x(2)*motor.a(2,1)+x(3)*motor.a(2,2)+motor.b(2)*uc;
        %As derived from the state space model,
        % y = theta_dot -> y is speed control
    endif
    
    % If enable is 1 run the MRAS 
    % This means updating the system to try and
    %determine teh current inertia of the system 
    if(enable == 1)
        %Use estimated inertia, J0
        %MRAS States
        px(4) = x(4)*model.a(1,1)+x(5)*model.a(1,2);
        px(5) = x(4)*model.a(2,1)+x(5)*model.a(2,2)+uc*model.b(2);

        %Update control signal based on theta
        %See example 5.1 pg. 187
        u = x(6)*uc;
        %Now implement plant with the modified 
        %control signal
        px(2) = x(2)*motor.a(1,1)+x(3)*motor.a(1,2);
        px(3) = x(2)*motor.a(2,1)+x(3)*motor.a(2,2)+u*motor.b(2);
    
        %Update MIT Rule
        % e = y - ym 
        e = x(2) - x(4);
        
        %Update input for next loop
        %In this case ym is a roational speed
        px(6) = -gamma*e*x(4);
    endif
endfunction;
