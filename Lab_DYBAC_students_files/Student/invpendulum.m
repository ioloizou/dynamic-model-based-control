

%% System Definition

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

x_0=[0.1; 0; 0.1; 0];
%% System control analysis

eigenvalues = eig(A);

contr=ctrb(sys);
c_rank = rank(contr);

%% Design of control

poles_desired = [-1+1i;-1-1i;-2+2*1i;-2-2*1i];

F = -acker(A, B, poles_desired);

%% Plotting

% x=out.x;
% tout=out.tout;
% u=out.u;
% 
% f1=figure(1) ;
% set(f1,'position',[1 305 672 500])
% subplot(321),plot(tout,x( :,1)),title('angle'),grid,
% subplot(323),plot(tout,x( :,2)),title('derivative of angle'),grid
% subplot(322),plot(tout,x( :,3)),title('position'),grid,
% subplot(324),plot(tout,x( :,4)),title('derivative of position'),grid
% subplot(325),plot(tout,u),title('control'),grid

%% Observer analysis
C=[1 0 0 0; 0 0 1 0;]; % C of the observer
obs=obsv(A,C);
obs_rank=rank(obs);

%% Observer design

poles_fast = [-100+1i, -100-1i, -200-2*1i, -200+2*1i];
K_fast=place(A.',C.',poles_fast).';
K=place(A.',C.',poles_desired).';