%% GNU Octave
%
% Octave example for MRAS following:
% Karl J. Astrom and Bjorn Wittenmark 
% Adapative Control 2nd edition Example 8.1
%
% Drew Currie Spring 2025
%
%PID Autotune Relay Oscillation

%Load packages
pkg load control
%Given system
K = 5;
alpha = 10;
d = 1;
uc = 0;
G = tf([K*alpha], [1 alpha+1, alpha, 0]);

%Calculate integrals
[y t] = step(G);

%Find k based on the max value of y since it is the height of y
k = max(y);
%A0 from figure 8.2
A0 = max(y)*max(t)-trapz(t,y);
%Find the index in t that is closed to the point of L + T
[M, I] = min(abs(t-(A0/k)));
%Determine A1
A1 = trapz(t(1:I), y(1:I));
%Determine T equation 8.7
T = (exp(1)*A1)/k;
L = (A0/k)-T;
%Determining alpha, figure 8.1
alpha = L*((0.63*k)/T);
%Pregenerate PID values
P = 0;
I = 0;
D = 0;
if(0)
    %Just proportional Control
    P = 1/alpha;
end
if(0)
    %PI Control
    P = 0.9/alpha;
    I = 3*L;
end 
if(1)
    %PID Control
    P = 1.2/alpha;
    I = 2*L;
    D = L/2;
end

%Create system to plot
C = pid(P, P/I, P*D);
T = feedback(tf(C)*G, 1);
step(T)