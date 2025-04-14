Ae = [0, 0, 0; 1, 0, 0; 0, 1, 0];
Be = [1, 0, 0];
Ce = [0, 1, 0];
B1 = 0.09;
B2 = 0.6;
b0 = 0.0001;
L0 = [B1, B2, 1]'/b0;

A = Ae-L0*Ce;
%Note this does not model y properly. 
%Still needs work see euqation 17 of SSPID paper
controller = ss(A, Be, Ce, 0);
