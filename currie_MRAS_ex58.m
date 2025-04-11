%% GNU Octave
%
% Octave example for MRAS following:
% Karl J. Astrom and Bjorn Wittenmark 
% Adapative Control 2nd edition Example 5.5
%
% Drew Currie Spring 2025
%
%Stability depends on the signal amplitudes example

clear; clc; close all;


function px = g(x,t,gamma)
 
  k0    = 2.0;        % model reference gain -> Given in example 5.1 reused in 5.5
  k     = 1.0;        %Underlying system gain -> Given in example
  
  %Create "square" wave input
  %Control signal amplitude
  frequency = 20/pi;
  shifted_t = t/frequency; 

  %Low amplitude low frequency signal is unstable
  if(0)
    amp = 0.1;
    uc = amp * (sin(shifted_t)+(sin(3*shifted_t)/3)+(sin(5*shifted_t)/5) + (sin(7*shifted_t)/7));
  end
  %Low frequency high amplitude signal is stable. Takes longer to reach a point where the system is matched. 
  if(1)
    amp = 1;
    uc = amp * (sin(shifted_t)+(sin(3*shifted_t)/3)+(sin(5*shifted_t)/5) + (sin(7*shifted_t)/7)+(sin(9*shifted_t)/9)...
    +(sin(11*shifted_t)/11)+(sin(13*shifted_t)/13) + (sin(15*shifted_t)/15)+sin(17*shifted_t)/17);
  end
  %High amplitude low frequency carrier with high frequency is not stable but results in an average
  if(0)
    amp = 3.5;
    uc = amp * (sin(shifted_t)+(sin(3*shifted_t)/3)+(sin(5*shifted_t)/5))+cos(0.5*shifted_t);
  end

  % model reference, state = x(1)
  px(1) = k0*uc - x(1);   % implement k0/(s+1)

  % controller and process, state = x(2)
  u = x(3)*uc;
  px(2) = k*u - x(2);   % implement k/(s+1)

  % MIT rule update
  %Update e
  e     = x(2)-x(1);
  %General MIT Rule
  if(0)
    px(3) = -gamma*e*x(1);   % see equation (5.5)
  end
  % Linearize MIT Rule
  % Implement equation 5.16
  % Alpha > 0 so help when phi is small
  if(1) 
    alpha = 0.0001/4;
    phi = -1*(k/k0)*px(1);
    px(4) = phi;
    px(3) = (-gamma *phi * e)/(alpha + phi*phi);
  end
endfunction;



% simulation 
t = linspace(0,150,2500);
%x(1) = ym 
%x(2) = y
%x(3) = d(theta)/dt
x0 = zeros(1,4);

gamma = 0.1;

x = lsode(@(x,t) g(x,t,gamma), x0, t);

figure(1);
plot(t,x(:,1),t,x(:,2));
xlabel('t');
ylabel('ym amd y');
legend('ym', 'y')

pause(0.25)
