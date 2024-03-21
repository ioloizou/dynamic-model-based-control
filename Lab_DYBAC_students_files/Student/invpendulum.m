clear all,close all,clear

%%System Definition

m=0.5; % mass of pendulum kg
M=5; % mass of the cart kg
L=1; % pendulum length meters
g=9.8; % gravitational accelaration m/s^2

A=[0 1 0 0;
   ((m+M)/M*L)*g 0 0 0;
   0 0 0 1;
   (-m*g/M) 0 0 0];

B=[0; -(1/M*L); 0; 1/M];

C=eye(4);

D=[0; 0; 0; 0];

sys=ss(A, B, C, D);
%% System control analysis

eigenvalues = eig(A);

contr=ctrb(sys);
c_rank = rank(contr);

%% Design of control

poles_desired = [-1+1i;-1-1i;-2+2*1i;-2-2*1i];

F_place = -place(A, B, poles_desired);

