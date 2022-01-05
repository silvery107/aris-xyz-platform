clear all;clc;close all
%Geometry Parameters
L1=0.4;L2=0.2;x=0.5;Lend=1;
theta_end_1=pi/3;
%Variance Buffer
theta3=0:0.005:sqrt(-x^2+(L1+L2)^2);
r3=[x*ones(1,size(theta3,2));-theta3];
r2=zeros(2,size(theta3,2));
r1=zeros(2,size(theta3,2));
rend=zeros(2,size(theta3,2));
Ls=zeros(1,size(theta3,2));
%Simulation
for i=1:size(theta3,2)
    % Known theta3 calculate joint2 position
    r2(:,i)=[x;-theta3(i)];
    Ls(i)=sqrt(r2(:,i)'*r2(:,i));
    [c1,c2,c3]=solve_triangle(L1,L2,Ls(i));
    theta0=c2+atan2(theta3(i),x);
    r1(:,i)=L1*[cos(theta0);-sin(theta0)];
    theta_end=theta0+theta_end_1;
    rend(:,i)=1/2*r1(:,i)+Lend*[cos(theta_end);-sin(theta_end)];
end
%Visualization
Visualization;
%Function
function [theta1,theta2,theta3]=solve_triangle(x1,x2,x3)
 theta1=acos((x2^2+x3^2-x1^2)/(2*x2*x3));
 theta2=acos((x1^2+x3^2-x2^2)/(2*x1*x3));
 theta3=acos((x1^2+x2^2-x3^2)/(2*x1*x2));
end