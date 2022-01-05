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
for i=1:size(r1,2)
    scatter(rend(1,i),rend(2,i),'LineWidth',5)
    hold on
    xlim([-1,1]);
    ylim([-2,0]);
    line([0,r1(1,i)],[0,r1(2,i)],'Linewidth',2);
    line([r1(1,i),r2(1,i)],[r1(2,i),r2(2,i)],'Linewidth',2);
    line([r2(1,i),x],[r2(2,i),0],'Linewidth',2);
    line([r1(1,i)/2,rend(1,i)],[r1(2,i)/2,rend(2,i)],'Linewidth',2)
    rectangle('Position',[0.058 -2 2 0.1+i*0.015],'FaceColor',[0 .5 .5],'EdgeColor','b',...
    'LineWidth',3,'Curvature',0.4)
    plot(rend(1,:),rend(2,:),'LineWidth',5)
    hold off
    drawnow;
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    if i == 1
        imwrite(I,map,'test.gif','gif','Loopcount',inf,'DelayTime',0.2);
    else
        imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0.2);
    end
    pause(0.2);
end
%Function
function [theta1,theta2,theta3]=solve_triangle(x1,x2,x3)
 theta1=acos((x2^2+x3^2-x1^2)/(2*x2*x3));
 theta2=acos((x1^2+x3^2-x2^2)/(2*x1*x3));
 theta3=acos((x1^2+x2^2-x3^2)/(2*x1*x2));
end