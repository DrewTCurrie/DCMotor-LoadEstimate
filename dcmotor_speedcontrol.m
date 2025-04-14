% Octave
% Created April 10th 2025
% Author: Drew Currie
% Created as the final project for MSU EELE-592
% This project will work to model the current load
% on a DC motor through the change in the moment of 
% inertia. 

function px = dcmotor_speedcontrol(x, t, enable, gamma, J0, motor)

    %x vector input map:
    %x(1) = theta_dot
    %x(2) = current in winding
    %x(3) = input voltage (terminal voltrage)
    %x(4) = Theta_dot_MRAS
    %x(5) = current_MRAS
    %x(6) = d(theta)/dt -> Control update signal for speed

    %Motor is expected to be a state space sys model
    %The A matrix is expected to be 2x2

    %System input, terminal voltage for the motor
    u = x(3);
    %Output vector
    px = zeros(1,6);
    % If enable is 0 do not run the MRAS
    % Just run the system in an open loop configuration
    if(enable == 0)
        %State 1, theta_dot
        px(1) = x(1)*motor.a(1,1)+x(2)*motor.a(1,2);
        px(2) = x(1)*motor.a(2,1)+x(2)*motor.a(2,2)+uc*motor.b(2);
        %As derived from the state space model,
        % y = theta_dot -> y is speed control
    endif
    % If enable is 1 run the MRAS 
    % This means updating the system to try and
    %determine teh current inertia of the system 
    if(enable == 1)
        %Use estimated inertia, J0
        %MRAS States
        px(4) = x(4)*motor.a(1,1)+x(5)*motor.a(1,2);
        px(5) = x(4)*motor.a(2,1)+x(5)*motor.a(2,2)+uc*motor.b(2);

        %Update control signal based on theta
        %See example 5.1 pg. 187
        u = x(6)*uc;
        %Now implement plant with the modified 
        %control signal
        px(1) = x(1)*motor.a(1,1)+x(2)*motor.a(1,2);
        px(2) = x(1)*motor.a(2,1)+x(2)*motor.a(2,2)+u*motor.b(2);
    
        %Update MIT Rule
        % e = y - ym 
        e = x(1) - x(4);
        
        %Update input for next loop
        %In this case ym is a roational speed
        px(6) = -gamma*e*x(4);
    endif
endfunction;
