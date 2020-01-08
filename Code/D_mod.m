clear all

%% Defining variables
syms m1 g m2 M L1 L2 x dx 
m1 = 100;
m2 = 100;
M = 1000;
L1 = 20;
L2 = 10;
g = 9.81;
tspan = 0:0.1:100;
% q = [x dx t1 dt1 t2 dt2];
%Enter initial conditions
q0 = [2 0 deg2rad(17) 0 deg2rad(30) 0];

%% Linearized Model
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
% step(sys,200);
eig(A-B*K);

%% Linear Model Controlled Response
[t,q1] = ode45(@(t,q)linear(t,q,-K*q),tspan,q0);
figure(1);
hold on
plot(t,q1(:,1))
plot(t,q1(:,3))
plot(t,q1(:,5))
ylabel('state variables')
xlabel('time (sec)')
title('Linear system using LQR controller')
legend('x','theta1','theta2')

%% Non_Linear Model Controlled Response
[t,q2] = ode45(@(t,q)nonLinear(t,q,-K*q),tspan,q0);
figure(2);
hold on
plot(t,q2(:,1))
plot(t,q2(:,3))
plot(t,q2(:,5))
ylabel('state variables')
xlabel('time (sec)')
title('Non-Linear system using LQR controller')
legend('x','theta1','theta2')