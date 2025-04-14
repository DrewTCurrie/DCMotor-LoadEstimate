% Octave
% Created April 10th 2025
% Author: Drew Currie
% Created as the final project for MSU EELE-592
% This project will work to model the current load
% on a DC motor through the change in the moment of 
% inertia. This system works to provide a 
% Model Reference Adpative Control for speed control 
% of the DC motor. 
%
% Call this function using an ODE solver such as lsode
%
% x(1) = input voltage  -> Terminal Voltage
% x(2) = theta          -> Motor Position (Unused)
% X(3) = theta_dot      -> Rotational speed
% x(4) = current        -> Winding current
% x(5) = theta_MRAS     -> MRAS Position (Unused)
% x(6) = Theta_dot_MRAS -> MRAS rotational speed
% x(7) = Current_MRAS   -> MRAS Widing Current
% x(8) = d(theta)/dt    -> Control update signal
%
% Motor is expected to be a state space sys model
% The system works off a modle for a DC motor with 
% position as a state. 
% 
% The A matrix is expected to be 3x3
% The B matrix is expected to be 1x3
% The C matrix is expected to be 3x1
% Model is expected to be a state space sys model
% The A matrix is expected to be 3x3 
% The A matrix should contain the desired behavior
% The B matrix is expected to be 1x3
% The C matrix is expected to be 3x1

function px = dcmotor_speedcontrol(x, t, enable, gamma, motor, model)

    %System input, terminal voltage for the motor
    uc = x(1);
    %Output vector
    px = zeros(1,8);
    % If enable is 0 do not run the MRAS
    % Just run the system in an open loop configuration
    if(enable == 0)
        %State 1, theta_dot
        px(3) = x(3)*motor.a(2,2)+x(4)*motor.a(2,3);
        px(4) = x(3)*motor.a(3,2)+x(4)*motor.a(3,3)+motor.b(3)*uc;
        %As derived from the state space model,
        % y = theta_dot -> y is speed control
    endif
    
    % If enable is 1 run the MRAS 
    % This means updating the system to try and
    %determine teh current inertia of the system 
    if(enable == 1)
        %Use estimated inertia, J0
        %MRAS States
        px(6) = x(6)*model.a(2,2)+x(7)*model.a(2,3);
        px(7) = x(6)*model.a(3,2)+x(7)*model.a(3,3)+model.b(3)*uc;

        %Update control signal based on theta
        %See example 5.1 pg. 187
        u = x(8)*uc;
        %Now implement plant with the modified 
        %control signal
        px(3) = x(3)*motor.a(2,2)+x(4)*motor.a(2,3);
        px(4) = x(3)*motor.a(3,2)+x(4)*motor.a(3,3)+motor.b(3)*u;
    
        %Update MIT Rule
        % e = y - ym 
        e = x(3) - x(6);
        
        %Update input for next loop
        %In this case ym is a roational speed
        px(8) = -gamma*e*x(6);
    endif
endfunction;
