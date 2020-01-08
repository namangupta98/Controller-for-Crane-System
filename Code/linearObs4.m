function dQe = linearObs4(t,Qe,Lue4)
m1 = 100;
m2 = 100;
M = 1000;
L1 = 20;
L2 = 10;
g = 9.81;
A = [0 1 0 0 0 0; 0 0 -m1*g/M 0 -m2*g/M 0; 0 0 0 1 0 0; 0 0 -((M*g)+(m1*g))/(M*L1) 0 -g*m2/(M*L1) 0; 0 0 0 0 0 1; 0 0 -m1*g/(M*L2) 0 -((M*g)+(m2*g))/(M*L2) 0];
B = [0; 1/M; 0; 1/(L1*M); 0; 1/(L2*M)];
c4 = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];
y4 = [Qe(1); Qe(3); Qe(5)];
K = 1; % feedback = 1;
dQe = (A+B*K)*Qe + Lue4*(y4 - c4*Qe);
end