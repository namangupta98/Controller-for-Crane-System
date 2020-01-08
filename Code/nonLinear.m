function dQ = nonLinear(t,y,F)
m1 = 100; m2 = 100; M = 1000; L1 = 20; L2 = 10; g = 9.81;
x = y(1);
dx = y(2);
t1 = y(3);
dt1 = y(4);
t2 = y(5);
dt2 = y(6);
dQ=zeros(6,1);
dQ(1) = dx;
dQ(2) = (F-((m1*sin(t1)*cos(t1))+(m2*sin(t2)*cos(t2)))*g - (L1*m1*(dQ(3)^2)*sin(t1)) - (L2*m2*(dQ(5)^2)*sin(t2)))/(m1+m2+M-(m1*(cos(t1)^2))-(m2*(cos(t2)^2)));
dQ(3) = dt1;
dQ(4) = (cos(t1)*dQ(2)-g*sin(t1))/L1;
dQ(5) = dt2;
dQ(6) = (cos(t2)*dQ(2)-g*sin(t2))/L2;
end