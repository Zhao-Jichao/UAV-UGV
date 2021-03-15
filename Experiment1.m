% 一阶仿真
clear
clc

% 目标运动轨迹
x0(1,1) = 10;
y0(1,1) = 5;
v0X(1,1) = 2;
v0Y(1,1) = 3.5;
theta0 = atan(v0Y/v0X);

% 时间参数
tBegin = 0;
tEnd   = 10;
dT     = 0.2;
times  = (tEnd-tBegin)/dT;
t(1,1) = 0;

% 跟踪者运动轨迹
x1(1,1) = 3;
y1(1,1) = 6;
z1(1,1) = 0;
% v1X = 0;
% v1Y = 0;
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

% 静态障碍物
xo = 20;
yo = 20;

% UAV
xa(1,1) = 2;
ya(1,1) = 0;
za(1,1) = 30;


% Parameters & Switches
figDynamic = 1;
fig1       = 1;
fig2       = 0;
fig3       = 0;

for time = 1:times
    % 记录目标轨迹
    x0(1,time+1) = x0(1,time) + dT * v0X;
    y0(1,time+1) = y0(1,time) + dT * v0Y;
    
    % 记录跟踪者轨迹
    K = 0.5;
    R = 5;
    
    dxo1 = abs(xo - x1(1,time));
    dyo1 = abs(yo - y1(1,time));
    
    % 判断是否受到斥力
    if dxo1 < 2
        Fxo1 = dxo1 * 2;
    else
        Fxo1 = 0;
    end
    if dyo1 < 2
        Fyo1 = dyo1 * 2;
    else
        Fyo1 = 0;
    end
    
    v1X = v0X + K * (x0(1,time) + R * cos(theta0) - x1(1,time)) + Fxo1;
    v1Y = v0Y + K * (y0(1,time) + R * sin(theta0) - y1(1,time)) + Fyo1;
    x1(1,time+1) = x1(1,time) + dT * v1X;
    y1(1,time+1) = y1(1,time) + dT * v1Y;
    z1(1,time+1) = z1(1,time) + dT * 0;
    
    v2X = 2 *2* K * (x0(1,time) + R * cos(theta0 + 0.5*pi) - x2(1,time));
    v2Y = 3 *2* K * (y0(1,time) + R * sin(theta0 + 0.5*pi) - y2(1,time));
    x2(1,time+1) = x2(1,time) + dT * v2X;
    y2(1,time+1) = y2(1,time) + dT * v2Y;
    
    v3X = v0X + K * (x0(1,time) + R * cos(theta0 + 1.0*pi) - x3(1,time));
    v3Y = v0Y + K * (y0(1,time) + R * sin(theta0 + 1.0*pi) - y3(1,time));
    x3(1,time+1) = x3(1,time) + dT * v3X;
    y3(1,time+1) = y3(1,time) + dT * v3Y;
    
    v4X = v0X + K * (x0(1,time) + R * cos(theta0 + 1.5*pi) - x4(1,time));
    v4Y = v0Y + K * (y0(1,time) + R * sin(theta0 + 1.5*pi) - y4(1,time));
    x4(1,time+1) = x4(1,time) + dT * v4X;
    y4(1,time+1) = y4(1,time) + dT * v4Y;
    
%     vaX = v0X + K * (x0(1,time) + R * cos(theta0 + 1.5*pi) - x4(1,time));
%     vaY = v0Y + K * (y0(1,time) + R * sin(theta0 + 1.5*pi) - y4(1,time));
    xa(1,time+1) = x4(1,time) + dT * v4X;
    ya(1,time+1) = y4(1,time) + dT * v4Y;
    za(1,time+1) = za(1,time) + dT * 0;
    
    % 记录误差距离
    d01(1,time+1) = sqrt( (x0(1,time+1)-x1(1,time+1))^2 + (y0(1,time+1)-y1(1,time+1))^2 );
    d02(1,time+1) = sqrt( (x0(1,time+1)-x2(1,time+1))^2 + (y0(1,time+1)-y2(1,time+1))^2 );
    d03(1,time+1) = sqrt( (x0(1,time+1)-x3(1,time+1))^2 + (y0(1,time+1)-y3(1,time+1))^2 );
    d04(1,time+1) = sqrt( (x0(1,time+1)-x4(1,time+1))^2 + (y0(1,time+1)-y4(1,time+1))^2 );
    
    % 记录误差距离
    d12(1,time+1) = sqrt( (x1(1,time+1)-x2(1,time+1))^2 + (y1(1,time+1)-y2(1,time+1))^2 );
    d23(1,time+1) = sqrt( (x2(1,time+1)-x3(1,time+1))^2 + (y2(1,time+1)-y3(1,time+1))^2 );
    d34(1,time+1) = sqrt( (x3(1,time+1)-x4(1,time+1))^2 + (y3(1,time+1)-y4(1,time+1))^2 );
    d41(1,time+1) = sqrt( (x4(1,time+1)-x1(1,time+1))^2 + (y4(1,time+1)-y1(1,time+1))^2 );
    
    % 记录时间
    t(1, time+1) = t(1,time) + dT;
    
    % 绘制动图
    if figDynamic == 1
        figure(1)
        plot3(x0,y0,z1,'>','color','r'); hold on
        plot3(x1,y1,z1,'o','color','g'); hold on
        plot3(x2,y2,z1,'o','color','b'); hold on
        plot3(x3,y3,z1,'o','color','m'); hold on
        plot3(x4,y4,za,'o','color','c'); hold on
        scatter(xo,yo); hold on

        legend('target 0','leader 1','follower 2','follower 3','follower 4');
        xlabel('X axis');
        ylabel('Y axis');
        axis([0,50, 0,50, 0,50]); 
        grid on;
    end    
end

% 绘制追踪过程图
if fig1 == 1
figure(1)
plot3(x0,y0,z1,'>','color','r'); hold on
plot3(x1,y1,z1,'o','color','g'); hold on
plot3(x2,y2,z1,'o','color','b'); hold on
plot3(x3,y3,z1,'o','color','m'); hold on
plot3(x4,y4,za,'o','color','c'); hold on
scatter(xo,yo); hold on
legend('target 0','leader 1','follower 2','follower 3','follower 4');
xlabel('X axis');
ylabel('Y axis');
axis([0,50, 0,50, 0,50]); 
grid on;

rectangle('Position',[x0(1,times+1)-R,y0(1,times+1)-R,2*R,2*R],...
            'linestyle','--','Curvature',[1,1],'linewidth',1);
title('Hunting Process');
end

% 绘制误差距
if fig2 == 1
figure(2)
plot(t,d01,'-','color','g'); hold on
plot(t,d02,'-','color','b'); hold on
plot(t,d03,'-','color','m'); hold on
plot(t,d04,'-','color','c'); hold on
legend('d01','d02','d03','d04');
xlabel('T/time');
ylabel('d/distance');
title('The Distance between the Target and Pursuers');
end

% 绘制误差距
if fig3 == 1
figure(3)
plot(t,d12,'-','color','g'); hold on
plot(t,d23,'-','color','b'); hold on
plot(t,d34,'-','color','m'); hold on
plot(t,d41,'-','color','c'); hold on
legend('d12','d23','d34','d41');
xlabel('T/time');
ylabel('d/distance');
title('The Distance between the Pursuers');
end