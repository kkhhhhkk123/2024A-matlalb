clc;close all;clear
warning off
luoju=55e-2; % 螺距
k=luoju/2/pi; % 螺线方程的系数 r=k theta
L1=341e-2;
D1=L1-27.5e-2*2; % 龙头把手两个孔之间的距离
L2=220e-2;
D2=L2-27.5e-2*2; % 其他凳子把手两个孔之间的距离

t=300;
N=223;
theta0=0:0.01:16*2*pi;
x=k*theta0.*cos(theta0);
y=k*theta0.*sin(theta0);

theta1=57.032076651015522;
dt=1;
time=300;
myfun=@(t,theta) -1./(k*sqrt(1+theta.^2));
X=zeros(N+1,1);
Y=zeros(N+1,1);
Theta=zeros(N+1,1);
Theta(1,1)=theta1;
X(1,1)=k*theta1*cos(theta1);
Y(1,1)=k*theta1*sin(theta1);
theta=[];
flag=0;
while flag==0
    [ttt,aaa]=ode45(myfun,[0 dt],Theta(1,1));
    Theta(1,1)=aaa(end);
    X(1,1)=k*Theta(1,1)*cos(Theta(1,1));
    Y(1,1)=k*Theta(1,1)*sin(Theta(1,1));
    for i=2:N+1
        d=D1*(i<=2)+D2*(i>2);
        Theta(i,1)=solve_theta(d,X(i-1,1),Y(i-1,1),Theta(i-1,1));
        X(i,1)=k*Theta(i,1)*cos(Theta(i,1));
        Y(i,1)=k*Theta(i,1)*sin(Theta(i,1));
    end
    theta=cat(2,theta,Theta);
    time=time+dt;
    figure(1)
    plot(x,y,'-');
    title({['t=',num2str(time)]});
    set(gcf,'Position',[200 200 600 600]);
    axis equal
    grid on
    xlabel('x')
    ylabel('y')
    hold on
    plot(X(:,1),Y(:,1),'k-','LineWidth',1.2,'Marker','o','MarkerSize',6,'MarkerFaceColor','r');
    drawnow
    hold off
    index1=find(Theta(1)+2*pi>=Theta);
    index2=find(Theta(2)+2*pi>=Theta);
    index=index1(end)-3:1:index2(end)+3;
    ttheta=zeros(4,1);
    ttheta(1)=Theta(1),ttheta(2)=Theta(2);
    for j=index(1):index(end-1)
       ttheta(3)=Theta(j),ttheta(4)=Theta(j+1);
       flag=if_collision(ttheta);
        if flag==1
            break;
        end
    end
end
v=k*sqrt(1+theta(:,end).^2).*(theta(:,end-1)-theta(:,end))./dt;

function theta=solve_theta(d,x1,y1,theta1) 
luoju=0.55;
k=luoju/2/pi;
fun=@(theta)(k*theta.*cos(theta)-x1).^2+(k*theta.*sin(theta)-y1).^2-d^2;
q=0.01;
options = optimoptions('fsolve','Display','off'); 
theta=fsolve(fun,theta1+q,options); 
while theta<=theta1 || abs(k*theta-k*theta1)>luoju/2 
    q=q+0.1;
    theta=fsolve(fun,theta+q,options); 
end 
end

