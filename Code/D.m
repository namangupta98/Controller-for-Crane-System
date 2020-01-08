%% Defining variables
syms m1 g m2 M L1 L2
m1 = 100;
m2 = 100;
M = 1000;
L1 = 20;
L2 = 10;
g = 9.81;

%% Linearized A and B
A = [0 1 0 0 0 0; 0 0 -m1*g/M 0 -m2*g/M 0; 0 0 0 1 0 0; 0 0 -((M*g)+(m1*g))/(M*L1) 0 -g*m2/(M*L1) 0; 0 0 0 0 0 1; 0 0 -m1*g/(M*L2) 0 -((M*g)+(m2*g))/(M*L2) 0];
B = [0; 1/M; 0; 1/(L1*M); 0; 1/(L2*M)];
c = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];
d = [1;0;0];
Rank = rank([B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B]);

%% LQR Controller
Q = [5 0 0 0 0 0; 0 0 0 0 0 0; 0 0 5000 0 0 0; 0 0 0 0 0 0; 0 0 0 0 5000 0; 0 0 0 0 0 0];
R = 0.001;
[K,S,P] = lqr(A,B,Q,R);
sys = ss(A-B*K,B,c,d);
step(sys,200);
eig(A-B*K);