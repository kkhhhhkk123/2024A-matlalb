load('data.mat');
%myVideo=VideoWriter('gradon')
open(myVideo)
for i=1:1:length(tt)
    plot(x,y,'-')
    set(gcf,'Position',[200 200 600 600])
    axis equal;
    grid on;
    xlabel('x');
    ylabel('y');
    hold on;

    plot(X1(:,i),Y1(:,i),'k','LineWidth',1.2,'Marker','o','MarkerSize',6,'MarkerFaceColor','r');
    title({['t=',num2str(tt(i))]});
    drawnow;
    %Fr=getframe(gcf);
    %writeVideo(myVideo,Fr)
    hold off
   
end
%close(myVideo)
%myVideo.FrameRate=10;
V=zeros(size(X1));
V(:,1)=-k*sqrt(1+Theta(:,1).^2).*(Theta(:,2)-Theta(:,1))./dt;
V(:,end)=-k*sqrt(1+Theta(:,end).^2).*(Theta(:,end)-Theta(:,end-1))/dt;
V(:,2:end-1)=-k*sqrt(1+Theta(:,2:end-1).^2).*(Theta(:,3:end)-Theta(:,1:end-2))/2/dt;

figure(5)
[TT,BB]=meshgrid(0:0.1:300,1:224);
pcolor(TT,BB,V);
shading interp;
set(gca,'FontSize',11);
xlabel('时间t');
ylabel('把手点的序号');
title('各个把手点的速度关于时间的分布');
colorbar;
