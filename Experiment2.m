% 二阶控制器设计
clear
clc

% 目标运动轨迹
x0(1,1) = 10;
y0(1,1) = 5;
vx0(1,1) = 2;
vy0(1,1) = 3.5;
theta0 = atan(vy0/vx0);
ux0(1,1) = 0;
uy0(1,1) = 0;

% 时间参数
tBegin = 0;
tEnd   = 10;
dT     = 0.1;
times  = (tEnd-tBegin)/dT;
t(1,1) = 0;

% 跟踪者运动轨迹
x1(1,1) = 3;
y1(1,1) = 6;
vx1(1,1) = 0;
vy1(1,1) = 0;
% theta1 = 0;


x2(1,1) = 1;
y2(1,1) = 15;

x3(1,1) = 0;
y3(1,1) = 3;

x4(1,1) = 2;
y4(1,1) = 0;

d01(1,1) = sqrt( (x0(1,1)-x1(1,1))^2 + (y0(1,1)-y1(1,1))^2 );
d02(1,1) = sqrt( (x0(1,1)-x2(1,1))^2 + (y0(1,1)-y2(1,1))^2 );
d03(1,1) = sqrt( (x0(1,1)-x3(1,1))^2 + (y0(1,1)-y3(1,1))^2 );
d04(1,1) = sqrt( (x0(1,1)-x4(1,1))^2 + (y0(1,1)-y4(1,1))^2 );

d12(1,1) = sqrt( (x1(1,1)-x2(1,1))^2 + (y1(1,1)-y2(1,1))^2 );
d23(1,1) = sqrt( (x2(1,1)-x3(1,1))^2 + (y2(1,1)-y3(1,1))^2 );
d34(1,1) = sqrt( (x3(1,1)-x4(1,1))^2 + (y3(1,1)-y4(1,1))^2 );
d41(1,1) = sqrt( (x4(1,1)-x1(1,1))^2 + (y4(1,1)-y1(1,1))^2 );

for time = 1:times
    % 记录目标轨迹
    vx0(1,time+1) = vx0(1,time) + dT * ux0(1,1);
    vy0(1,time+1) = vy0(1,time) + dT * uy0(1,1);
    x0(1,time+1) = x0(1,time) + dT * vx0(1,time+1);
    y0(1,time+1) = y0(1,time) + dT * vy0(1,time+1);
    
    % 记录跟踪者轨迹----------------
    K = 0.5;
    J = 0.8;
    R = 5;
    
    % 控制协议选择----------------
    protocol = 2;
    switch protocol
    case 1
        u1X = ux0 + K * (vx0(1,time) - vx1(1,time) + R * cos(theta0));
        u1Y = uy0 + K * (vy0(1,time) - vy1(1,time) + R * sin(theta0));
    case 2
        u1X = ux0 - K * (x0(1,time) - x1(1,time) + R * cos(theta0))...
            + J * (vx0(1,time) - vx1(1,time));
        
        u1Y = uy0 - K * (y0(1,time) - y1(1,time) + R * sin(theta0))...
            + J * (vy0(1,time) - vy1(1,time));
    end
    
    
%     v1X = v0X + K * (x0(1,time) + R * cos(theta0) - x1(1,time));
%     v1Y = v0Y + K * (y0(1,time) + R * sin(theta0) - y1(1,time));
    vx1(1,time+1) = vx1(1,time) + dT * u1X;
    vy1(1,time+1) = vy1(1,time) + dT * u1Y;    
    x1(1,time+1)  = x1(1,time)  + dT * vx1(1,time+1);
    y1(1,time+1)  = y1(1,time)  + dT * vy1(1,time+1);
    
%     v2X = v0X + K * (x0(1,time) + R * cos(theta0 + 0.5*pi) - x2(1,time));
%     v2Y = v0Y + K * (y0(1,time) + R * sin(theta0 + 0.5*pi) - y2(1,time));
%     x2(1,time+1) = x2(1,time) + dT * v2X;
%     y2(1,time+1) = y2(1,time) + dT * v2Y;
%     
%     v3X = v0X + K * (x0(1,time) + R * cos(theta0 + 1.0*pi) - x3(1,time));
%     v3Y = v0Y + K * (y0(1,time) + R * sin(theta0 + 1.0*pi) - y3(1,time));
%     x3(1,time+1) = x3(1,time) + dT * v3X;
%     y3(1,time+1) = y3(1,time) + dT * v3Y;
%     
%     v4X = v0X + K * (x0(1,time) + R * cos(theta0 + 1.5*pi) - x4(1,time));
%     v4Y = v0Y + K * (y0(1,time) + R * sin(theta0 + 1.5*pi) - y4(1,time));
%     x4(1,time+1) = x4(1,time) + dT * v4X;
%     y4(1,time+1) = y4(1,time) + dT * v4Y;
    
%     % 记录误差距离
    d01(1,time+1) = sqrt( (x0(1,time+1)-x1(1,time+1))^2 + (y0(1,time+1)-y1(1,time+1))^2 );
%     d02(1,time+1) = sqrt( (x0(1,time+1)-x2(1,time+1))^2 + (y0(1,time+1)-y2(1,time+1))^2 );
%     d03(1,time+1) = sqrt( (x0(1,time+1)-x3(1,time+1))^2 + (y0(1,time+1)-y3(1,time+1))^2 );
%     d04(1,time+1) = sqrt( (x0(1,time+1)-x4(1,time+1))^2 + (y0(1,time+1)-y4(1,time+1))^2 );
%     
%     % 记录误差距离
%     d12(1,time+1) = sqrt( (x1(1,time+1)-x2(1,time+1))^2 + (y1(1,time+1)-y2(1,time+1))^2 );
%     d23(1,time+1) = sqrt( (x2(1,time+1)-x3(1,time+1))^2 + (y2(1,time+1)-y3(1,time+1))^2 );
%     d34(1,time+1) = sqrt( (x3(1,time+1)-x4(1,time+1))^2 + (y3(1,time+1)-y4(1,time+1))^2 );
%     d41(1,time+1) = sqrt( (x4(1,time+1)-x1(1,time+1))^2 + (y4(1,time+1)-y1(1,time+1))^2 );
    
    % 记录时间
    t(1, time+1) = t(1,time) + dT;
    
    % 绘制动图
    figure(1)
    subplot(2,1,1)
    plot(x0,y0,'>','color','r'); hold on
    plot(x1,y1,'o','color','g'); hold on
%     plot(x2,y2,'o','color','b'); hold on
%     plot(x3,y3,'o','color','m'); hold on
%     plot(x4,y4,'o','color','c'); hold on
    xlabel('Time');
    ylabel('Position');
    title('Position-Time');
    
    subplot(2,1,2)
    plot(vx0,vy0,'>','color','r'); hold on
    plot(vx1,vy1,'o','color','g'); hold on
    xlabel('Time');
    ylabel('Speed');
    title('Speed-Time');
    
end

% rectangle('Position',[x0(1,times+1)-R,y0(1,times+1)-R,2*R,2*R],'linestyle','--','Curvature',[1,1],'linewidth',1);
% title('Hunting Process');

% % 绘制误差距
% figure(2)
% plot(t,d01,'-','color','g'); hold on
% plot(t,d02,'-','color','b'); hold on
% plot(t,d03,'-','color','m'); hold on
% plot(t,d04,'-','color','c'); hold on
% legend('d01','d02','d03','d04');
% xlabel('T/time');
% ylabel('d/distance');
% title('The Distance between the Target and Pursuers');
% 
% % 绘制误差距
% figure(3)
% plot(t,d12,'-','color','g'); hold on
% plot(t,d23,'-','color','b'); hold on
% plot(t,d34,'-','color','m'); hold on
% plot(t,d41,'-','color','c'); hold on
% legend('d12','d23','d34','d41');
% xlabel('T/time');
% ylabel('d/distance');
% title('The Distance between the Pursuers');
