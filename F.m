clear all

%% Defining variables
syms m1 g m2 M L1 L2
m1 = 100;
m2 = 100;
M = 1000;
L1 = 20;
L2 = 10;
g = 9.81;
q0 = [2 0 deg2rad(17) 0 deg2rad(30) 0];
tspan = 0:0.1:100;

%% Observability Check
A = [0 1 0 0 0 0; 0 0 -m1*g/M 0 -m2*g/M 0; 0 0 0 1 0 0; 0 0 -((M*g)+(m1*g))/(M*L1) 0 -g*m2/(M*L1) 0; 0 0 0 0 0 1; 0 0 -m1*g/(M*L2) 0 -((M*g)+(m2*g))/(M*L2) 0];
B = [0; 1/M; 0; 1/(L1*M); 0; 1/(L2*M)];
c1 = [1 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
c2 = [0 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];
c3 = [1 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 1 0];
c4 = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];
d = [0; 0; 0];
Obs1 = rank([c1' A'*c1' ((A')^2)*c1' ((A')^3)*c1' ((A')^4)*c1' ((A')^5)*c1']);
Obs2 = rank([c2' A'*c2' ((A')^2)*c2' ((A')^3)*c2' ((A')^4)*c2' ((A')^5)*c2']);
Obs3 = rank([c3' A'*c3' ((A')^2)*c3' ((A')^3)*c3' ((A')^4)*c3' ((A')^5)*c3']);
Obs4 = rank([c4' A'*c4' ((A')^2)*c4' ((A')^3)*c4' ((A')^4)*c4' ((A')^5)*c4']);

sys1 = ss(A,B,c1,d);
sys3 = ss(A,B,c3,d);
sys4 = ss(A,B,c4,d);
% step(sys1);
% step(sys3,300)
% step(sys4);
% eig(A-B*K);

%% Kalman Estimator Design
Bd = 0.1*eye(6);                %Process Noise
Vn = 0.01;                      %Measurement Noise
[Lue1,P,E] = lqe(A,Bd,c1,Bd,Vn*eye(3));
[Lue3,P,E] = lqe(A,Bd,c3,Bd,Vn*eye(3));
[Lue4,P,E] = lqe(A,Bd,c4,Bd,Vn*eye(3));

Ac1 = A-(Lue1*c1);
Ac3 = A-(Lue3*c3);
Ac4 = A-(Lue4*c4);
e_sys1 = ss(Ac1,[B Lue1],c1,0);
e_sys3 = ss(Ac3,[B Lue3],c3,0);
e_sys4 = ss(Ac4,[B Lue4],c4,0);
% step(e_sys1)
% step(e_sys3)
% step(e_sys4)

%% Generating plot for step input
unitStep = 0*tspan;
unitStep(200:length(tspan)) = 1;

[y1,t] = lsim(sys1,unitStep,tspan);
[x1,t] = lsim(e_sys1,[unitStep;y1'],tspan);

[y3,t] = lsim(sys3,unitStep,tspan);
[x3,t] = lsim(e_sys3,[unitStep;y3'],tspan);

[y4,t] = lsim(sys4,unitStep,tspan);
[x4,t] = lsim(e_sys4,[unitStep;y4'],tspan);

figure();
hold on
plot(t,y1(:,1),'r','Linewidth',2)
plot(t,x1(:,1),'k--','Linewidth',1)
ylabel('State Variables')
xlabel('time(sec)')
legend('x(t)','Estimated x(t)')
title('Response for output vector at step input: (x(t)')
hold off

figure();
hold on
plot(t,y3(:,1),'r','Linewidth',2)
plot(t,y3(:,3),'b','Linewidth',2)
plot(t,x3(:,1),'k--','Linewidth',1)
plot(t,x3(:,3),'m--','Linewidth',1)
ylabel('State Variables')
xlabel('time(sec)')
legend('x(t)','theta_2(t)','Estimated x(t)','Estimated theta_2(t)')
title('Response for output vector at step input: (x(t),theta_2(t))')
hold off

figure();
hold on
plot(t,y4(:,1),'r','Linewidth',2)
plot(t,y4(:,2),'g','Linewidth',2)
plot(t,y4(:,3),'b','Linewidth',2)
plot(t,x4(:,1),'k--','Linewidth',1)
plot(t,x4(:,2),'r--','Linewidth',1)
plot(t,x4(:,3),'m--','Linewidth',1)
ylabel('State Variables')
xlabel('time(sec)')
legend('x(t)','theta_1(t)','theta_2(t)','Estimated x(t)','Estimated theta_1(t)','Estimated theta_2(t)')
title('Response for output vector at step input: (x(t),theta_1(t),theta_2(t))')
hold off

%%
%% Linear Model Observer Response
[t,q1] = ode45(@(t,q)linearObs1(t,q,Lue1),tspan,q0);
figure();
hold on
plot(t,q1(:,1))
ylabel('state variables')
xlabel('time (sec)')
title('Linear system Observer for output vector: x(t)')
legend('x')
hold off

[t,q3] = ode45(@(t,q)linearObs3(t,q,Lue3),tspan,q0);
figure();
hold on
plot(t,q3(:,1))
plot(t,q3(:,5))
ylabel('state variables')
xlabel('time (sec)')
title('Linear system Observer for output vector: (x(t),theta_2(t))')
legend('x','theta_2')
hold off

[t,q4] = ode45(@(t,q)linearObs4(t,q,Lue4),tspan,q0);
figure();
hold on
plot(t,q4(:,1))
plot(t,q4(:,3))
plot(t,q4(:,5))
ylabel('state variables')
xlabel('time (sec)')
title('Linear system Observer for output vector: (x(t),theta_1(t),theta_2(t))')
legend('x','theta_1','theta_2')
hold off

%%
%% Non-linear Model Observer Response
[t,q1] = ode45(@(t,q)nonLinearObs1(t,q,1,Lue1),tspan,q0);
figure();
hold on
plot(t,q1(:,1))
ylabel('state variables')
xlabel('time (sec)')
title('Non-Linear System Observer for output vector: x(t)')
legend('x')
hold off

[t,q3] = ode45(@(t,q)nonLinearObs3(t,q,1,Lue3),tspan,q0);
figure();
hold on
plot(t,q3(:,1))
plot(t,q3(:,5))
ylabel('state variables')
xlabel('time (sec)')
title('Non-Linear System Observer for output vector: (x(t),theta_2(t))')
legend('x','theta_2')
hold off

[t,q4] = ode45(@(t,q)nonLinearObs4(t,q,1,Lue4),tspan,q0);
figure();
hold on
plot(t,q4(:,1))
plot(t,q4(:,3))
plot(t,q4(:,5))
ylabel('state variables')
xlabel('time (sec)')
title('Non-Linear System Observer for output vector: (x(t),theta_1(t),theta_2(t))')
legend('x','theta_1','theta_2')
hold off