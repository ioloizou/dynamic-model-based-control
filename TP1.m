%% parameters
m = 0.5;
M = 5;
L = 1;
g = 9.8;

%% State-Space:
A = [0 1 0 0 ; ((M+m)*g)/(M*L) 0 0 0; 0 0 0 1; (-m*g)/M 0 0 0];
B = [0; -1/M*L;0;1/M];
C_block = eye(4);
D = [0;0;0;0];

lambda = eig(A);

%% Controllability analysis:
Cont = ctrb(A,B);
c_rank = rank(Cont);

%% Static Feedback
p_desired = [-1+1j, -1-1j, -2+2*1j, -2-2*1j];
F = -place(A,B,p_desired);
x_0 = [0.1;0;0.1;0];    %initial conditions

%% Observability Analysis:
C = [1 0 0 0; 0 0 1 0];
O = obsv(A,C);
o_rank = rank(O);

p_fast = [-100+1i, -100-1i, -200-2*1i, -200+2*1i];
K_fast = place(A.',C.',p_fast).'; 
K = place(A.',C.', p_desired).';
%% Simulate
sim('tp1_feedback',10);

%% plot
% x1 = out.x(:,1);
% x2 = out.x(:,2);
% x3 = out.x(:,3);
% x4 = out.x(:,4);
% 
% x_hat1 = out.x_hat(:,1);
% x_hat2 = out.x_hat(:,2);
% x_hat3 = out.x_hat(:,3);
% x_hat4 = out.x_hat(:,4);
% 
% u = out.u;
% 
% f1=figure(1) ;
% set(f1,'position',[1 305 672 500])
% subplot(321),plot(out.tout,x1),title('angle'),grid,
% subplot(321),plot(out.tout,x1),title('angle'),grid
% 
% subplot(323),plot(out.tout,x2),title('derivative of angle'),grid
% subplot(323),plot(out.tout,x2),title('derivative of angle'),grid
% 
% subplot(322),plot(out.tout,x3),title('position'),grid,
% subplot(322),plot(out.tout,x3),title('position'),grid,
% 
% subplot(324),plot(out.tout,x4),title('derivative of position'),grid
% subplot(324),plot(out.tout,x4),title('derivative of position'),grid
% 
% subplot(325),plot(out.tout,u),title('control'),grid

