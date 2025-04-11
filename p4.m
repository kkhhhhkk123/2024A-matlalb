clc;close all;clear
warning off
luoju=1.7; % 螺距
k=luoju/2/pi; % 螺线方程的系数 r=k theta
L1=341e-2;
D1=L1-27.5e-2*2; % 龙头把手两个孔之间的距离
L2=220e-2;
D2=L2-27.5e-2*2; % 其他凳子把手两个孔之间的距离
v0=1; % 头节点速度
R=4.5; %掉头区域半径
%%
Theta_ru=R/k,x_ru=k*Theta_ru*cos(Theta_ru),y_ru=k*Theta_ru*sin(Theta_ru); %盘入点的角度、坐标
K_ru=(sin(Theta_ru)+Theta_ru*cos(Theta_ru))/(cos(Theta_ru)-Theta_ru*sin(Theta_ru)); %切线方程k
B_ru=y_ru-K_ru*x_ru;   %盘入切点的切线系数b
tan_k=atan(K_ru); %切线的角度
d1=2*abs(B_ru/sqrt(1+K_ru^2));  %盘入盘出的切线之间的距离
cos_alph=d1/(2*R);  %等腰三角形的底角的余弦值
alph=acos(cos_alph);   %等腰三角形底角值
Theta1=pi-2*alph;   %等腰三角形的顶角
R2=R/(3*cos_alph),R1=2*R2;  %大小圆弧半径

x_c1=x_ru+R1*cos(tan_k+pi/2),y_c1=y_ru+R1*sin(tan_k+pi/2)  %大圆弧圆心坐标
Theta_chu=Theta_ru-pi,x_chu=R*cos(Theta_chu),y_chu=R*sin(Theta_chu); %盘出点的角度、坐标
K_chu=K_ru;
x_c2=x_chu-R2*cos(tan_k+pi/2),y_c2=y_chu-R2*sin(tan_k+pi/2); %小圆弧圆心坐标

Theta_ru_c1=angle((x_ru-x_c1)+1i*(y_ru-y_c1))+2*pi; %盘入点与c1圆心连线的角度
theta_range1=linspace(Theta_ru_c1,Theta_ru_c1-Theta1,50); %c1的角度范围
X_c1=R1*cos(theta_range1)+x_c1;
Y_c1=R1*sin(theta_range1)+y_c1;
Theta_chu_c2=angle((x_chu-x_c2)+1i*(y_chu-y_c2));
theta_range2=linspace(Theta_chu_c2,Theta_chu_c2-Theta1,50);
X_c2=R2*cos(theta_range2)+x_c2;
Y_c2=R2*sin(theta_range2)+y_c2;
%%
figure(1)
hold on  %两端圆弧和圆心
plot(x_c1,y_c1,'r*');
h1=plot(X_c1,Y_c1,'r','LineWidth',1.3);
plot(x_c2,y_c2,'b*');
h2=plot(X_c2,Y_c2,'b','Linewidth',1.3);

theta1=5*2*pi:-0.01:Theta_ru;
r1=k*theta1;
x1=r1.*cos(theta1),y1=r1.*sin(theta1);
set(gcf,'Position',[100,100,600,600]);
c1=rand(1,3);
h3=plot(x1,y1,'Color',c1,'LineWidth',1.3);%盘入螺线
grid on;

theta2=theta1-pi;
r2=k*(theta2+pi);
x2=r2.*cos(theta2),y2=r2.*sin(theta2);
c2=rand(1,3);
h4=plot(x2,y2,'Color',c2,'LineWidth',1.3);%盘出螺线

theta3=0:0.01:2*pi;
x3=R*cos(theta3),y3=R*sin(theta3);
c3=rand(1,3);
h5=plot(x3,y3,'Color',c3,'LineWidth',1.3); %掉头区域
legend([h1,h2,h3,h4,h5],{'圆弧c1','圆弧c2','盘入螺线','盘出螺线','掉头区域'});
%a1=R1*cos(Theta_ru_c1-Theta1)+x_c1,a2=R1*sin(Theta_ru_c1-Theta1)+y_c1;
%b1=R2*cos(Theta_chu_c2-Theta1)+x_c2,b2=R2*sin(Theta_chu_c2-Theta1)+y_c2;
%%
%计算龙头的运动
caltheta=@(t,theta) 1./(k*sqrt(theta.^2+1));
dt=0.1;
t_rang=0:0.1:100;
[tt,theta]=ode45(caltheta,t_rang,Theta_ru);
THeta=zeros(224,2001);
XX=zeros(224,2001);
YY=zeros(224,2001);
%在盘入螺线上运动
THeta(1,1:length(theta))=theta(end:-1:1);
XX(1,1:length(theta))=k*THeta(1,1:length(theta)).*cos(THeta(1,1:length(theta)));
YY(1,1:length(theta))=k*THeta(1,1:length(theta)).*sin(THeta(1,1:length(theta))); 
%在圆弧c1上运动
t1=dt:dt:R1*Theta1;%在圆弧运动的时间
THeta(1,length(theta)+(1:length(t1)))=-t1*v0/R1+Theta_ru_c1;
XX(1,length(theta)+(1:length(t1)))=R1*cos(THeta(1,length(theta)+(1:length(t1))))+x_c1;
YY(1,length(theta)+(1:length(t1)))=R1*sin(THeta(1,length(theta)+(1:length(t1))))+y_c1;
%在圆弧c2上运动
t2=0:dt:R2*Theta1;
THeta(1,length(theta)+length(t1)+(1:length(t2)))=t2*v0/R2+Theta_chu_c2-Theta1;
XX(1,length(theta)+length(t1)+(1:length(t2)))=R2*cos(THeta(1,length(theta)+length(t1)+(1:length(t2))))+x_c2;
YY(1,length(theta)+length(t1)+(1:length(t2)))=R2*sin(THeta(1,length(theta)+length(t1)+(1:length(t2))))+y_c2;
%在盘出螺线上
caltheta2=@(t,theta)1./(k*sqrt(1+(theta+pi).^2));
t_rang2=0:dt:(100-Theta1*(R1+R2)/v0);
[tt2,theta_2]=ode45(caltheta2,t_rang2,Theta_chu);
THeta(1,length(theta)+length(t1)+length(t2)+(1:length(tt2)))=theta_2;
XX(1,length(theta)+length(t1)+length(t2)+(1:length(tt2)))=k*(theta_2+pi).*cos(theta_2);
YY(1,length(theta)+length(t1)+length(t2)+(1:length(tt2)))=k*(theta_2+pi).*sin(theta_2);
%%
figure(2)
% for i=1:length(XX(1,:))
%     plot(XX(1,i),YY(1,i),'Marker','o','MarkerSize',3,'MarkerFaceColor','r');
%     hold on
%     drawnow
%     axis equal;
%     set(gcf,'Position',[200 200 600 600]);
% end
t_now=-100:dt:100;
hwait=waitbar(0,"开始计算");
%%
for i=1:(200/dt+1)
%  if THeta(1,i)>=Theta_ru && (THeta(1,i)>THeta(1,i+1)||THeta(1,i)<THeta(1,i-1))
    if t_now(i)<=0
        flag=1;
        for j=2:224
            d=D1*(j<=2)+D2*(j>2);
            TH=solve_theta1(THeta(j-1,i),XX(j-1,i),YY(j-1,i),d);
            THeta(j,i)=TH;
            XX(j,i)=k*TH*cos(TH);
            YY(j,i)=k*TH*sin(TH);
        end
    elseif t_now(i)>0 && (THeta(1,i)>=Theta_ru_c1-Theta1 && THeta(1,i)<=Theta_ru_c1)
        flag=2;
        for j=2:224
            d=D1*(j<=2)+D2*(j>2);
            if flag==2 %前一个结点在c1上
                [TH flag]=solve_theta2(THeta(j-1,i),XX(j-1,i),YY(j-1,i),d,Theta_ru_c1,Theta_ru,R1,k);
                THeta(j,i)=TH;
                if flag==2  %两个结点都在c1上
                    XX(j,i)=R1*cos(TH)+x_c1;
                    YY(j,i)=R1*sin(TH)+y_c1;
                else   %前一个结点在c1，下一点在盘入螺线中
                    XX(j,i)=k*TH*cos(TH);
                    YY(j,i)=k*TH*sin(TH);
                end
            else   %前后两个结点都在盘入螺线上
                TH=solve_theta1(THeta(j-1,i),XX(j-1,i),YY(j-1,i),d);
                THeta(j,i)=TH;
                XX(j,i)=k*TH*cos(TH);
                YY(j,i)=k*TH*sin(TH);
            end
        end
    elseif t_now(i)>R1*Theta1 && (THeta(1,i)<=Theta_chu_c2 && THeta(1,i)>=Theta_chu_c2-Theta1)
        flag=3;
        for j=2:224
            d=D1*(j<=2)+D2*(j>2);
            if flag==3 %前一个结点在c2上
                [TH flag]=solve_theta3(THeta(j-1,i),XX(j-1,i),YY(j-1,i),d,Theta_chu_c2,Theta_ru_c1,Theta1,R1,R2,x_c1,y_c1);
                THeta(j,i)=TH;
                if flag==3  %前后都在c2上
                    XX(j,i)=R2*cos(TH)+x_c2;
                    YY(j,i)=R2*sin(TH)+y_c2;
                else flag==2 %前一个在c2，后一个在c1
                    XX(j,i)=R1*cos(TH)+x_c1;
                    YY(j,i)=R1*sin(TH)+y_c1;
                end
            elseif flag==2 %前一个结点在c1上
                [TH flag]=solve_theta2(THeta(j-1,i),XX(j-1,i),YY(j-1,i),d,Theta_ru_c1,Theta_ru,R1,k);
                THeta(j,i)=TH;
                if flag==2 %前后都在c1
                    XX(j,i)=R1*cos(TH)+x_c1;
                    YY(j,i)=R1*sin(TH)+y_c1;
                else  %前一个在c1，后一个在盘入螺线上
                    XX(j,i)=k*TH*cos(TH);
                    YY(j,i)=k*TH*sin(TH);
                end
            else %前后都在盘入螺线上
                TH=solve_theta1(THeta(j-1,i),XX(j-1,i),YY(j-1,i),d);
                THeta(j,i)=TH;
                XX(j,i)=k*TH*cos(TH);
                YY(j,i)=k*TH*sin(TH);
            end
        end
    else
        flag=4;
        for j=2:224
            d=D1*(j<=2)+D2*(j>2);
            if flag==4 %前一个在盘出螺线上
                [TH flag]=solve_theta4(THeta(j-1,i),XX(j-1,i),YY(j-1,i),d,Theta_chu,Theta_chu_c2,R2,x_c2,y_c2,k);
                THeta(j,i)=TH;
                if flag==4 %前后都在盘出螺线上
                    XX(j,i)=k*(TH+pi)*cos(TH);
                    YY(j,i)=k*(TH+pi)*sin(TH);
                else %前一个在盘出螺线，后一个在c2上
                    XX(j,i)=R2*cos(TH)+x_c2;
                    YY(j,i)=R2*sin(TH)+y_c2;
                end
            elseif flag==3 %前一个在c2上
                [TH flag]=solve_theta3(THeta(j-1,i),XX(j-1,i),YY(j-1,i),d,Theta_chu_c2,Theta_ru_c1,Theta1,R1,R2,x_c1,y_c1);
                THeta(j,i)=TH;
                if flag==3  %前后都在c2
                    XX(j,i)=R2*cos(TH)+x_c2;
                    YY(j,i)=R2*sin(TH)+y_c2;
                else  %前一个在c2,后一个在c1
                    XX(j,i)=R1*cos(TH)+x_c1;
                    YY(j,i)=R1*sin(TH)+y_c1;
                end
            elseif flag==2 %前一个结点在c1
                [TH flag]=solve_theta2(THeta(j-1,i),XX(j-1,i),YY(j-1,i),d,Theta_ru_c1,Theta_ru,R1,k);
                THeta(j,i)=TH;
                if flag==2 %前后都在c1
                    XX(j,i)=R1*cos(TH)+x_c1;
                    YY(j,i)=R1*sin(TH)+y_c1;
                else %前一个在c1，后一个在盘入螺线上
                    XX(j,i)=k*TH*cos(TH);
                    YY(j,i)=k*TH*sin(TH);
                end
            else %前后都在盘入螺线
                TH=solve_theta1(THeta(j-1,i),XX(j-1,i),YY(j-1,i),d);
                THeta(j,i)=TH;
                XX(j,i)=k*TH*cos(TH);
                YY(j,i)=k*TH*sin(TH);
            end
        end
    end
%    elseif (THeta(1,i)>=Theta_ru_c1-Theta1 && THeta(1,i)<=Theta_ru_c1) && (THeta(1,i)<THeta(1,i-1)||(THeta(1,i)>THeta(1,i+1)))
%    elseif (THeta(1,i)>=Theta_chu_c2-Theta1 && THeta(1,i)<=Theta_chu_c2) && (THeta(1,i)>THeta(1,i-1)||(THeta(1,j)<THeta(1,i-1)))
%    else (THeta(1,i)>=Theta_chu) && (THeta(1,i)<(THeta(1,i+1))||THeta(1,i)>THeta(1,i-1))
    jindu=100*i/2001;
    waitbar(i/(200/dt+1),hwait,[num2str(i),'/2001 ',num2str(jindu),'%']);
end
%%
figure(3)
for j=(1:2:2001)
    plot(XX(:,j),YY(:,j),'k-','LineWidth',1.2,'Marker','o','MarkerSize',6,'MarkerFaceColor','r');
    hold on;
    plot(x_c1,y_c1,'r*');
    plot(x_c2,y_c2,'b*');
    plot(x1,y1,'Color',c1,'LineWidth',1.3);
    plot(x2,y2,'Color',c2,'LineWidth',1.3);
    plot(X_c1,Y_c1,'r','LineWidth',1.3);
    plot(X_c2,Y_c2,'b','Linewidth',1.3);
    title({['t=',num2str(dt*(j-1)-100)]})
    set(gcf,'Position',[300,100,600,600]);
    axis equal;
    grid on;
    xlabel('x');
    ylabel('y');
    axis([-15 15 -15 15]);
    drawnow;
    hold off;
end
vx=zeros(size(XX));
vy=zeros(size(XX));
v=zeros(size(YY));
vx(:,1)=(XX(:,1)-XX(:,2))/dt;
vx(:,end)=(XX(:,end)-XX(:,end-1))/dt;
vx(:,2:end-1)=(XX(:,1:end-2)-XX(:,3:end))/2/dt;
vy(:,1)=(YY(:,1)-YY(:,2))/dt;
vy(:,end)=(YY(:,end)-YY(:,end-1))/dt;
vy(:,2:end-1)=(YY(:,1:end-2)-YY(:,3:end))/2/dt;
v=sqrt(vx.^2+vy.^2);
for i=1:2001
    figure(4)
    ylabel("V");
    plot(1:224,v(:,i));
    v_max=max(v(:,i));
    v_min=min(v(:,i));
    q=(v_max-v_min)/2;
    ylim([v_min-q,v_max+q]);
    yline(1,'r--');
    ylabel("v");
    drawnow;
    tt=-100+(i-1)*dt;
    title(['t= ',num2str(tt),' s']); 
end

figure(5)
[TT,BB]=meshgrid(-100:0.1:100,1:224);
pcolor(TT,BB,v);
shading interp;
set(gca,'FontSize',11);
xlabel('时间t');
ylabel('把手点的序号');
title('各个把手点的速度关于时间的分布');
colorbar;
%%
function theta= solve_theta1(ptheta,px,py,d) %两个点都在盘入螺线上
q=0;
luoju=1.7;
k=luoju/2/pi;
options = optimoptions('fsolve','Display','off');
fun=@(theta)(px-k*theta.*cos(theta)).^2+(py-k*theta.*sin(theta)).^2-d^2;
theta=fsolve(fun,ptheta+q,options);
while theta<=ptheta || abs(theta-ptheta)>pi
    q=q+0.1;
    theta=fsolve(fun,ptheta+q,options);
end
end

function [theta flag]=solve_theta2(ptheta,px,py,d,THeta_ru_c1,Theta_ru,R1,k)
delt_theta=2*asin((d/2)/R1);
theta=ptheta+delt_theta;
flag=2;
if theta>=THeta_ru_c1;
    flag=1;
    theta=solve_theta1(4.5/k,px,py,d);
end
end

function [theta flag]=solve_theta3(ptheta,px,py,d,Theta_chu_c2,Theta_ru_c1,Theta1,R1,R2,x_c1,y_c1)
delt_theta=2*asin((d/2)/R2);
theta=ptheta-delt_theta;
flag=3;
if theta<=Theta_chu_c2-Theta1
    flag=2;
    q=0;
    d1=sqrt((px-x_c1)^2+(py-y_c1)^2);
    cos_delt_theta=(R1^2+d1^2-d^2)/2/R1/d1;
    delt_theta=acos(cos_delt_theta);
    theta=atan((py-y_c1)/(px-x_c1))+delt_theta;
    %caltheta4=@(theta)(px-(R1*cos(theta)+x_c1)).^2+(py-(R1*sin(theta)+y_c1)).^2-d^2;
    %options=optimoptions('fsolve','Display','none');
    %theta=fsolve(caltheta4,Theta_ru_c1-Theta1,options);
    %while theta<Theta_ru_c1-Theta1
    %    q=q+0.1;
    %    theta=fsolve(caltheta4,Theta_ru_c1-Theta1+q,options);
    %end
end
end

function [theta flag]=solve_theta4(ptheta,px,py,d,Theta_chu,Theta_chu_c2,R2,x_c2,y_c2,k)
q=0;
flag=4;
%options = optimoptions('fsolve','Display','off');
%fun=@(theta)(px-k*(theta+pi).*cos(theta)).^2+(py-k*(theta+pi).*sin(theta)).^2-d^2;

%theta_linjie_fun=@(t)(k*(Theta_chu+pi)*cos(Theta_chu)-k*(t+pi).*cos(t)).^2+(k*(Theta_chu+pi)*sin(Theta_chu)-k*(t+pi).*sin(t)).^2-d^2;
%options = optimoptions('fsolve','Display','off');
%t=fsolve(theta_linjie_fun,Theta_chu,options);
%while t<=Theta_chu || abs(t-Theta_chu)>pi
%    q=q+0.1;
%    t=fsolve(theta_linjie_fun,Theta_chu+q,options);
%end

theta=solve_theta5(ptheta,px,py,k,d)
%if ptheta>=t
if theta>Theta_chu
    theta=theta;
    %flag=4;
    %q=0;
    %theta=fsolve(fun,ptheta);
    %while (theta>=ptheta ||theta<=Theta_chu) || abs(theta-ptheta)>pi
    %   q=q-0.1;
    %   theta=fsolve(fun,ptheta+q);
    %end
else  %到了圆弧c2上
    flag=3;
    %d1=norm([x_c2-px,y_c2-py]);
    d1=sqrt((x_c2-px)^2+(y_c2-py)^2);
    theta_r2_point=angle((x_c2-px)+1i*(y_c2-py));
    delt_theta_cos=(d1^2+R2^2-d^2)/(2*d1*R2);
    delt_theta=acos(delt_theta_cos);
    theta=theta_r2_point-delt_theta+pi;
    %q=0;
    %FUN=@(theta)(px-(R2*cos(theta)+x_c2)).^2+(py-(R2*sin(theta)+y_c2)).^2-d^2;
    %theta=fsolve(FUN,Theta_chu_c2);
    %while theta>=Theta_chu_c2 || abs(theta-Theta_chu_c2)>pi
    %    q=q-0.01;
    %    theta=fsolve(FUN,Theta_chu_c2+q);
    %end
end
end

function theta=solve_theta5(ptheta,px,py,k,d)
q=-0.1;
fun=@(theta)(px-k*(theta+pi).*cos(theta)).^2+(py-k*(theta+pi).*sin(theta)).^2-d^2;
options = optimoptions('fsolve','Display','off');
theta=fsolve(fun,ptheta+q,options);
while theta>ptheta || abs(theta-ptheta)>pi
    q=q-0.1;
    theta=fsolve(fun,ptheta+q,options);
end
end