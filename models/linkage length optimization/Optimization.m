clear all;clc;close all
global xw R Lend theta_end
xw=0.3;
R=0.5;
Lend=1;
theta_end=pi/3;
A=[-1,-1;1,-1;0,1];
b=[-xw;xw;xw];
lb=[0,0];
ub=[0.7,xw];
x0=[0.5,0.2];
[x,error]=fmincon(@(x)forward_kinematic(x),x0,A,b,[],[],lb,ub);
function e=forward_kinematic(x)
    global xw R theta_end Lend
    L1=x(1);
    L2=x(2);
    theta0=atan2(sqrt(L1^2-(xw-L2)^2),xw);
    xend=xw-L1/2*cos(theta0)-Lend*cos(theta0+theta_end);
    e=abs(xend-R);
end