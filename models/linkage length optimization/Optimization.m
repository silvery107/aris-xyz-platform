clear all;clc;close all
global xw R Lend theta_end
xw=0.4;
R=0.5;
Lend=1;
theta_end=pi/6;
lb=[0,0];
ub=[1.5,1.5];
x0=[0.4,0.2];
[x,error]=fmincon(@(x)forward_kinematic(x),x0,[],[],[],[],lb,ub,@limf);
function [g,h]=limf(x)
    global xw
    g=[-x(1)-x(2)+xw;...
        3*x(1)^2-3*xw^2-5*x(2)^2-2*x(1)*x(2)+8*xw*x(2);...
        x(1)-x(2)-xw];
    h=[];
end
function e=forward_kinematic(x)
    global xw R theta_end Lend
    L1=x(1);
    L2=x(2);
    theta0=atan2(sqrt(L1^2-(xw-L2)^2),xw);
    xend=xw-L1/2*cos(theta0)-Lend*cos(theta0+theta_end);
    e=abs(xend-R);
end