close all;
% transformation
t=1;
r1=r1-[t;0];
r2=r2-[t;0];
x=x-t;
rend=rend-[t;0];
rend2=rend2-[t;0];
for i=1:size(r1,2)
    scatter([rend(1,i),-rend(1,i)],[rend(2,i),rend(2,i)],'LineWidth',5)
    hold on
    axis off
    xlim([-1,1]);
    ylim([-2,0]);
    line([-t,r1(1,i)],[0,r1(2,i)],'Linewidth',2);
    line([r1(1,i),r2(1,i)],[r1(2,i),r2(2,i)],'Linewidth',2);
    line([r2(1,i),x],[r2(2,i),0],'Linewidth',2);
    line([(r1(1,i)-t)/2,rend2(1,i)],[r1(2,i)/2,rend2(2,i)],'Linewidth',5)
    line([rend(1,i),rend2(1,i)],[rend(2,i),rend2(2,i)],'Linewidth',5)
    line([t,-r1(1,i)],[0,r1(2,i)],'Linewidth',2);
    line([-r1(1,i),-r2(1,i)],[r1(2,i),r2(2,i)],'Linewidth',2);
    line([-r2(1,i),-x],[r2(2,i),0],'Linewidth',2);
    line([-(r1(1,i)-t)/2,-rend2(1,i)],[r1(2,i)/2,rend2(2,i)],'Linewidth',5)
    line([-rend(1,i),-rend2(1,i)],[rend(2,i),rend2(2,i)],'Linewidth',5)
    rectangle('Position',[-0.5 -2.5 1 0.6+i*0.02],'FaceColor',[0.6350 0.0780 0.1840],'EdgeColor','r',...
    'LineWidth',3,'Curvature',0.4)
%     plot(rend(1,:),rend(2,:),'LineWidth',5)
%     plot(-rend(1,:),rend(2,:),'LineWidth',5)
    hold off
    drawnow;
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    if i == 1
        imwrite(I,map,'test.gif','gif','Loopcount',inf,'DelayTime',0.1);
    elseif rend(1,i-1)<rend(1,i)&&rend(1,i)>0.5-t-0.01&&rend(1,i)<0.5-t+0.01
        j=0;
        while(j<5)
            imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0.1);
            pause(0.2);
            j=j+1;
        end
        break;
    else
        imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0.1);
    end
    pause(0.2);
end