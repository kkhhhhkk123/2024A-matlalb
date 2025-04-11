clc,close all,clear
theta0=16*2*pi;
L1=341e-2;
L2=220e-2;
D1=L1-27.5e-2*2;
D2=L2-27.5e-2*2;
luoju=0.55;
k=luoju/2/pi;

theta=theta0:-0.01:0;
r=k*theta;
x=r.*cos(theta),y=r.*sin(theta);
figure(1)
plot(x,y,'-');
set(gcf,'Position',[200 200 600 600]);
axis equal
grid on
xlabel('x')
ylabel('y')
hold on
fun=@(t,theta)-1./(k*sqrt(1+theta^2))
dt=0.1;
tt=0:dt:300;
[t,theta]=ode45(fun,tt,theta0);
X=k*theta.*cos(theta);
Y=k*theta.*sin(theta);
for i=1:10:length(theta)
    plot(X(i),Y(i),'r.','MarkerSize',10);
    title({['t=',num2str(tt(i))]});
    drawnow;
end
N=223;
X1=zeros(N+1,length(tt));
Y1=zeros(N+1,length(tt));
Theta=zeros(N+1,length(tt));
X1(1,:)=X;
Y1(1,:)=Y;
Theta(1,:)=theta;
cwait=waitbar(0,"正在计算...");
for i=1:length(tt)
    for j=2:N+1
        d=D1*(j<=2)+D2*(j>2);
        thetaij=solve_theta(Theta(j-1,i),X1(j-1,i),Y1(j-1,i),d);
        Theta(j,i)=thetaij;
        X1(j,i)=k*thetaij*cos(thetaij);
        Y1(j,i)=k*thetaij*sin(thetaij);
        c=100*((i-1)*N+j-1)/(length(tt)*N);
        waitbar(((i-1)*N+j-1)/(length(tt)*N),cwait,['已经完成..',num2str((i-1)*N+j-1),'/670000  ',num2str(c),'%'])
    end
end
close(cwait);
save('data.mat');

function theta= solve_theta(ptheta,px,py,d)
q=0.1;
luoju=0.55;
k=luoju/2/pi;
options = optimoptions('fsolve','Display','off');
fun=@(theta)(px-k*theta.*cos(theta)).^2+(py-k*theta.*sin(theta)).^2-d^2;
theta=fsolve(fun,ptheta+q,options);
while theta<=ptheta || abs(k*theta-k*ptheta)>luoju/2
    q=q+0.1;
    theta=fsolve(fun,ptheta+q,options);
end
end