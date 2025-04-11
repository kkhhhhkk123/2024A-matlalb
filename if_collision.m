function flag= if_collision(ttheta)
luoju=55e-2; % 螺距
k=luoju/2/pi; % 螺线方程的系数 r=k theta
d=0.3;
flag=0;
L1=341e-2;
D1=L1-27.5e-2*2; % 龙头把手两个孔之间的距离
L2=220e-2;
D2=L2-27.5e-2*2; % 其他凳子把手两个孔之间的距离
x=k.*ttheta.*cos(ttheta),y=k.*ttheta.*sin(ttheta);
p1_c=[x(1)+x(2) y(1)+y(2)]/2,p2_c=[x(3)+x(4) y(3)+y(4)]/2;
k1=(x(1)-x(2))/(y(2)-y(1)),K1=-1/k1;
k2=(x(3)-x(4))/(y(4)-y(3)),K2=-1/k2;
A=[1 -k1;
    1 -k2];
B=[p1_c(2)-k1*p1_c(1);p2_c(2)-k2*p2_c(1)];
p_now=(inv(A)*B)';
p_now=p_now(end:-1:1);
X1=p1_c-p_now,X2=p2_c-p_now;
d1=norm(X1),d2=norm(X2);
theta1=angle(X1(1)+X1(2)*1i),theta2=angle(X2(1)+X2(2)*1i);
theta=abs(theta2-theta1);
T=[cos(theta) -sin(theta);
    sin(theta) cos(theta)];
[X,Y]=meshgrid(linspace(d1-d/2,d1+d/2,10),linspace(-L1/2,L1/2,10));
XY_new=T*[X(:),Y(:)]';
%a=find(abs(XY_new(:,1)-d2)=<0.15 & abs(XY_new(:,2))=<L2/2);
% for i=1:1:100
%     if abs(XY_new(1,i)-d2)<=d/2&abs(XY_new(2,i)<=L2/2);
%         flag=1;
%         break;
%     end
% end
z=find(abs(XY_new(1,:)-d2)<=d/2&abs(XY_new(2,:))<=L2/2);
if ~isempty(z)
    flag=1;
 %   [x,y]=meshgrid(linspace(d2-d/2,d2+d/2,10),linspace(-L2/2,L2/2,10));
 %   x=x(:),y=y(:);
 %   figure(2)
 %   scatter(XY_new(1,:),XY_new(2,:),'r','filled');
 %   hold on
 %   scatter(x(:),y(:),'y','filled');
    vex_1=[XY_new(1,1),XY_new(1,10),XY_new(1,100),XY_new(1,91)];
    vey_1=[XY_new(2,1),XY_new(2,10),XY_new(2,100),XY_new(2,91)];
    vex_2=[d2-d/2,d2+d/2,d2+d/2,d2-d/2];
    vey_2=[-L2/2,-L2/2,L2/2,L2/2];
    figure(2)
    hold on;
    fill(vex_1,vey_1,'r');
    fill(vex_2,vey_2,'y');
    legend("龙头","龙身");
    axis equal;
    grid on;
    hold off;
else
    flag=0;
end
end

