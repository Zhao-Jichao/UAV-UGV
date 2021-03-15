% 避障控制
clear
clc

% 目标运动轨迹
p0(:,1)     = [10;5];  
theta0(:,1) = 0;
v0(1,1)     = 0;


% UAV 运动轨迹
pA(:,1)     = [0;0]; 
vA(:,1)     = [0*cos(0);0*sin(0)]; 
thetaA(1,1) = 0;

% UGV 运动轨迹
p1(:,1)     = [3; 6];
v1(:,1)     = [0; 0]; 
theta1(1,1) = 0;

p2(:,1)     = [1; 15];
v2(:,1)     = [0; 0]; 
theta2(1,1) = 0;

p3(:,1)     = [0; 3];
v3(:,1)     = [0; 0]; 
theta3(1,1) = 0;

p4(:,1)     = [2; 0];
v4(:,1)     = [0; 0]; 
theta4(1,1) = 0;

% 时间参数
tBegin = 0;
tEnd   = 10;
dT     = 0.01;
times  = (tEnd-tBegin)/dT;
t(1,1) = 0;

% 其他关键参数
K = 1;          % 速度观测器
kappa = 0.6;

alpha = 0.15;      % 一致性参数
beta  = 0.1;
gamma = 1.0;      % 追捕参数
gasp  = 0.8;

R = 10;         % 追捕半径

% 障碍物参数
po = [15; 25];
Rd = 10;
Rs = 5;        % 影响范围
Rs2 =2;         % 障碍物直径

xid1 = 0;
xid2 = 0;
xid3 = 0;
xid4 = 0;

xis1 = 0.0000;
% xis2 = 0.027065;  % 保存参数
xis2 = 0.001;
xis3 = 0.00;
xis4 = 0.0000;

strategyType = 2;
avoidanceDynamic = 0;
avoidanceStatic = 0;


huitu  = 0;     % 速度观测器
huitu2 = 0;
huitu3 = 0;
huitu4 = 1;     % 四种策略
huitu5 = 0;     % 避障效果分析
huitu6 = 0;     % 避障控制效果图

% 绘制 UGV 三维图辅助信息
z1(1,1) = 0;
zA(1,1) = 0;

for time = 1:times
    
    if strategyType == 1
        % 1. 静止不动
        theta0(1,time)   = 1;
        v0(1,1)  = 0;
        u0       = 0;
    end
    
    if strategyType == 2
        % 2. 匀速直线运动
        theta0(1,time)   = 1;
        v0(1,1)  = 4;
        u0       = 0;
    end
    
    if strategyType == 3
        % 3. 最近追捕者的速度方向
        d01 = sqrt((p0(1,time)-p1(1,time)).^2+(p0(2,time)-p1(2,time)).^2);
        d02 = sqrt((p0(1,time)-p2(1,time)).^2+(p0(2,time)-p2(2,time)).^2);
        d03 = sqrt((p0(1,time)-p3(1,time)).^2+(p0(2,time)-p3(2,time)).^2);
        d04 = sqrt((p0(1,time)-p4(1,time)).^2+(p0(2,time)-p4(2,time)).^2);
        if d01 <= d02 && d01 <= d03 && d01 <= d04
            theta0(1,time)   = theta1(1,time);
        end
        if d02 <= d03 && d02 <= d04 && d02 <= d01
            theta0(1,time)   = theta2(1,time);
        end
        if d03 <= d04 && d03 <= d01 && d03 <= d02
            theta0(1,time)   = theta3(1,time);
        end
        if d04 <= d01 && d04 <= d02 && d04 <= d03
            theta0(1,time)   = theta4(1,time);
        end
        v0(1,1)  = 4;
        u0       = 0;
    end
    
    if strategyType == 4
        % 4. 所有追捕者的标准化速度矢量和
        theta1(1,time) = atan(v1(2,time)./(v1(1,time)+0.000001));
        theta2(1,time) = atan(v2(2,time)./(v2(1,time)+0.000001));
        theta3(1,time) = atan(v3(2,time)./(v3(1,time)+0.000001));
        theta4(1,time) = atan(v4(2,time)./(v4(1,time)+0.000001));
        theta0(1,time) = atan( (sin(theta1(1,time))+sin(theta2(1,time))+sin(theta3(1,time))+sin(theta4(1,time)))...
                              /(cos(theta1(1,time))+cos(theta2(1,time))+cos(theta3(1,time))+cos(theta4(1,time))) );
        v0(1,1)  = 4;
        u0       = 0;
    end
    
    % 目标的轨迹
    v0(1,time+1) = v0(1,time) + dT * u0;
    p0(1,time+1) = p0(1,time) + dT * v0(1,time) * cos(theta0(1,time));
    p0(2,time+1) = p0(2,time) + dT * v0(1,time) * sin(theta0(1,time));
    theta0(1,time+1) = theta0(1,time) + dT * 0;
    
    % UAV 的轨迹，需要传递观测到的速度
    vA(:,time+1) = K * (p0(:,time)-pA(:,time)) + kappa * vA(:,time);
    pA(:,time+1) = pA(:,time) + dT * vA(:,time+1);

    % 避障控制
    F1(:,time) = [0;0];
    F2(:,time) = [0;0];
    F3(:,time) = [0;0];
    F4(:,time) = [0;0];
    % 动态障碍物
    if avoidanceDynamic == 1
        d12 = sqrt( (p1(1,time)-p2(1,time))^2+(p1(2,time)-p2(2,time))^2 );
        d13 = sqrt( (p1(1,time)-p3(1,time))^2+(p1(2,time)-p3(2,time))^2 );
        d14 = sqrt( (p1(1,time)-p4(1,time))^2+(p1(2,time)-p4(2,time))^2 );
        d23 = sqrt( (p2(1,time)-p3(1,time))^2+(p2(2,time)-p3(2,time))^2 );
        d24 = sqrt( (p2(1,time)-p4(1,time))^2+(p2(2,time)-p4(2,time))^2 );
        d34 = sqrt( (p3(1,time)-p4(1,time))^2+(p3(2,time)-p4(2,time))^2 );

        if d12 <= Rd
            p12 = abs([p1(1,time)-p2(1,time); p1(2,time)-p2(2,time)]);
            F1(1,time) = F1(1,time) + (- (2*Rd - 2*p12(1,:))/(Rd + p12(1,:))^2 - (2*(Rd - p12(1,:))^2)/(Rd + p12(1,:))^3);
            F1(2,time) = F1(2,time) + (- (2*Rd - 2*p12(2,:))/(Rd + p12(2,:))^2 - (2*(Rd - p12(2,:))^2)/(Rd + p12(2,:))^3);
            F2(1,time) = F2(1,time) - (- (2*Rd - 2*p12(1,:))/(Rd + p12(1,:))^2 - (2*(Rd - p12(1,:))^2)/(Rd + p12(1,:))^3);
            F2(2,time) = F2(2,time) - (- (2*Rd - 2*p12(2,:))/(Rd + p12(2,:))^2 - (2*(Rd - p12(2,:))^2)/(Rd + p12(2,:))^3);
        end
        if d13 <= Rd
            p13 = abs([p1(1,time)-p3(1,time); p1(2,time)-p3(2,time)]);
            F1(1,time) = F1(1,time) + (- (2*Rd - 2*p13(1,:))/(Rd + p13(1,:))^2 - (2*(Rd - p13(1,:))^2)/(Rd + p13(1,:))^3);
            F1(2,time) = F1(2,time) + (- (2*Rd - 2*p13(2,:))/(Rd + p13(2,:))^2 - (2*(Rd - p13(2,:))^2)/(Rd + p13(2,:))^3);
            F3(1,time) = F3(1,time) - (- (2*Rd - 2*p13(1,:))/(Rd + p13(1,:))^2 - (2*(Rd - p13(1,:))^2)/(Rd + p13(1,:))^3);
            F3(2,time) = F3(2,time) - (- (2*Rd - 2*p13(2,:))/(Rd + p13(2,:))^2 - (2*(Rd - p13(2,:))^2)/(Rd + p13(2,:))^3);
        end
        if d14 <= Rs
            p14 = abs([p1(1,time)-p4(1,time); p1(2,time)-p4(2,time)]);
            F1(1,time) = F1(1,time) + (- (2*Rd - 2*p14(1,:))/(Rd + p14(1,:))^2 - (2*(Rd - p14(1,:))^2)/(Rd + p14(1,:))^3);
            F1(2,time) = F1(2,time) + (- (2*Rd - 2*p14(2,:))/(Rd + p14(2,:))^2 - (2*(Rd - p14(2,:))^2)/(Rd + p14(2,:))^3);
            F4(1,time) = F4(1,time) - (- (2*Rd - 2*p14(1,:))/(Rd + p14(1,:))^2 - (2*(Rd - p14(1,:))^2)/(Rd + p14(1,:))^3);
            F4(2,time) = F4(2,time) - (- (2*Rd - 2*p14(2,:))/(Rd + p14(2,:))^2 - (2*(Rd - p14(2,:))^2)/(Rd + p14(2,:))^3);
        end
        if d23 <= Rs
            p23 = abs([p2(1,time)-p3(1,time); p2(2,time)-p3(2,time)]);
            F2(1,time) = F2(1,time) + (- (2*Rd - 2*p23(1,:))/(Rd + p23(1,:))^2 - (2*(Rd - p23(1,:))^2)/(Rd + p23(1,:))^3);
            F2(2,time) = F2(2,time) + (- (2*Rd - 2*p23(2,:))/(Rd + p23(2,:))^2 - (2*(Rd - p23(2,:))^2)/(Rd + p23(2,:))^3);
            F3(1,time) = F3(1,time) - (- (2*Rd - 2*p23(1,:))/(Rd + p23(1,:))^2 - (2*(Rd - p23(1,:))^2)/(Rd + p23(1,:))^3);
            F3(2,time) = F3(2,time) - (- (2*Rd - 2*p23(2,:))/(Rd + p23(2,:))^2 - (2*(Rd - p23(2,:))^2)/(Rd + p23(2,:))^3);
        end
        if d24 <= Rs
            p24 = abs([p2(1,time)-p4(1,time); p2(2,time)-p4(2,time)]);
            F2(1,time) = F2(1,time) + (- (2*Rd - 2*p24(1,:))/(Rd + p24(1,:))^2 - (2*(Rd - p24(1,:))^2)/(Rd + p24(1,:))^3);
            F2(2,time) = F2(2,time) + (- (2*Rd - 2*p24(2,:))/(Rd + p24(2,:))^2 - (2*(Rd - p24(2,:))^2)/(Rd + p24(2,:))^3);
            F4(1,time) = F4(1,time) - (- (2*Rd - 2*p24(1,:))/(Rd + p24(1,:))^2 - (2*(Rd - p24(1,:))^2)/(Rd + p24(1,:))^3);
            F4(2,time) = F4(2,time) - (- (2*Rd - 2*p24(2,:))/(Rd + p24(2,:))^2 - (2*(Rd - p24(2,:))^2)/(Rd + p24(2,:))^3);
        end
        if d34 <= Rs
            p34 = abs([p3(1,time)-p4(1,time); p3(2,time)-p4(2,time)]);
            F3(1,time) = F3(1,time) + (- (2*Rd - 2*p34(1,:))/(Rd + p34(1,:))^2 - (2*(Rd - p34(1,:))^2)/(Rd + p34(1,:))^3);
            F3(2,time) = F3(2,time) + (- (2*Rd - 2*p34(2,:))/(Rd + p34(2,:))^2 - (2*(Rd - p34(2,:))^2)/(Rd + p34(2,:))^3);
            F4(1,time) = F4(1,time) - (- (2*Rd - 2*p34(1,:))/(Rd + p34(1,:))^2 - (2*(Rd - p34(1,:))^2)/(Rd + p34(1,:))^3);
            F4(2,time) = F4(2,time) - (- (2*Rd - 2*p34(2,:))/(Rd + p34(2,:))^2 - (2*(Rd - p34(2,:))^2)/(Rd + p34(2,:))^3);
        end
        F1(:,time) = F1(:,time) * xid1;
        F2(:,time) = F2(:,time) * xid2;
        F3(:,time) = F3(:,time) * xid3;
        F4(:,time) = F4(:,time) * xid4;
    end
    % 静态障碍物
    if avoidanceStatic == 1
        d1o = sqrt( (p1(1,time)-po(1,   1))^2+(p1(2,time)-po(2,   1))^2 );
        d2o = sqrt( (p2(1,time)-po(1,   1))^2+(p2(2,time)-po(2,   1))^2 );
        d3o = sqrt( (p3(1,time)-po(1,   1))^2+(p3(2,time)-po(2,   1))^2 );
        d4o = sqrt( (p4(1,time)-po(1,   1))^2+(p4(2,time)-po(2,   1))^2 );
        if d1o <= Rs
            p1o = ([p1(1,time)-po(1,   1); p1(2,time)-po(2,   1)]);
            F1(1,time) = F1(1,time) + (- (2*Rs - 2*p1o(1,:))/(Rs + p1o(1,:))^2 - (2*(Rs - p1o(1,:))^2)/(Rs + p1o(1,:))^3);
            F1(2,time) = F1(2,time) + (- (2*Rs - 2*p1o(2,:))/(Rs + p1o(2,:))^2 - (2*(Rs - p1o(2,:))^2)/(Rs + p1o(2,:))^3);
        end
        if d2o <= Rs
            p2o = [p2(1,time)-po(1,   1); p2(2,time)-po(2,   1)];
            F2(1,time) = F2(1,time) + (- (2*Rs - 2*p2o(1,:))/(Rs + p2o(1,:))^2 - (2*(Rs - p2o(1,:))^2)/(Rs + p2o(1,:))^3);
            F2(2,time) = F2(2,time) + (- (2*Rs - 2*p2o(2,:))/(Rs + p2o(2,:))^2 - (2*(Rs - p2o(2,:))^2)/(Rs + p2o(2,:))^3);
        end
        if d3o <= Rs
            p3o = [p3(1,time)-po(1,   1); p3(2,time)-po(2,   1)];
            F3(1,time) = F3(1,time) + (- (2*Rs - 2*p3o(1,:))/(Rs + p3o(1,:))^2 - (2*(Rs - p3o(1,:))^2)/(Rs + p3o(1,:))^3);
            F3(2,time) = F3(2,time) + (- (2*Rs - 2*p3o(2,:))/(Rs + p3o(2,:))^2 - (2*(Rs - p3o(2,:))^2)/(Rs + p3o(2,:))^3);
        end
        if d4o <= Rs
            p4o = [p4(1,time)-po(1,   1); p4(2,time)-po(2,   1)];
            F4(1,time) = F4(1,time) + (- (2*Rs - 2*p4o(1,:))/(Rs + p4o(1,:))^2 - (2*(Rs - p4o(1,:))^2)/(Rs + p4o(1,:))^3);
            F4(2,time) = F4(2,time) + (- (2*Rs - 2*p4o(2,:))/(Rs + p4o(2,:))^2 - (2*(Rs - p4o(2,:))^2)/(Rs + p4o(2,:))^3);
        end
        F1(:,time) = F1(:,time) * xis1;
        F2(:,time) = F2(:,time) * xis2;
        F3(:,time) = F3(:,time) * xis3;
        F4(:,time) = F4(:,time) * xis4;
    end

    % UGV 的轨迹
    u11 = alpha * ( (p2(:,time)-p1(:,time))+(p3(:,time)-p1(:,time))+(p4(:,time)-p1(:,time)) );
    u12 = beta  * ( (v2(:,time)-v1(:,time))+(v3(:,time)-v1(:,time))+(v4(:,time)-v1(:,time)) );
    u13 = gamma * (  p0(:,time)-p1(:,time) + R * [cos(theta0(1,time) + (1-1)*0.5*pi);sin(theta0(1,time) + (1-1)*0.5*pi)]);
    u14 = gasp    * (  vA(:,time+1)        -v1(:,time));
    u15 = xis1 * F1(:,time);
%     disp ([u11 u12 u13 u14 u15]) ;
    u1(:,time) = alpha * ( (p2(:,time)-p1(:,time))+(p3(:,time)-p1(:,time))+(p4(:,time)-p1(:,time)) )...
       + beta  * ( (v2(:,time)-v1(:,time))+(v3(:,time)-v1(:,time))+(v4(:,time)-v1(:,time)) )...
       + gamma * (  p0(:,time)-p1(:,time) + R * [cos(theta0(1,time) + (1-1)*0.5*pi);sin(theta0(1,time) + (1-1)*0.5*pi)])...
       + gasp    * (  vA(:,time+1)        -v1(:,time))...
       + F1(:,time);
   
    u2(:,time) = alpha * ( (p1(:,time)-p2(:,time))+(p3(:,time)-p2(:,time))+(p4(:,time)-p2(:,time)) )...
       + beta  * ( (v1(:,time)-v2(:,time))+(v3(:,time)-v2(:,time))+(v4(:,time)-v2(:,time)) )...
       + gamma * (  p0(:,time)-p2(:,time) + R * [cos(theta0(1,time) + (2-1)*0.5*pi);sin(theta0(1,time) + (2-1)*0.5*pi)])...
       + gasp    * (  vA(:,time+1)        -v2(:,time))...
       + F2(:,time);
       
    u3(:,time) = alpha * ( (p1(:,time)-p3(:,time))+(p2(:,time)-p3(:,time))+(p4(:,time)-p3(:,time)) )...
       + beta  * ( (v1(:,time)-v3(:,time))+(v2(:,time)-v3(:,time))+(v4(:,time)-v3(:,time)) )...
       + gamma * (  p0(:,time)-p3(:,time) + R * [cos(theta0(1,time) + (3-1)*0.5*pi);sin(theta0(1,time) + (3-1)*0.5*pi)])...
       + 1    * (  vA(:,time+1)        -v3(:,time))...
       + F3(:,time);
       
    u4(:,time) = alpha * ( (p1(:,time)-p4(:,time))+(p2(:,time)-p4(:,time))+(p3(:,time)-p4(:,time)) )...
       + beta  * ( (v1(:,time)-v4(:,time))+(v2(:,time)-v4(:,time))+(v3(:,time)-v4(:,time)) )...
       + gamma * (  p0(:,time)-p4(:,time) + R * [cos(theta0(1,time) + (4-1)*0.5*pi);sin(theta0(1,time) + (4-1)*0.5*pi)])...
       + gasp    * (  vA(:,time+1)        -v4(:,time))...
       + F4(:,time);
       
    v1(:,time+1) = v1(:,time) + dT * u1(:,time);
    p1(:,time+1) = p1(:,time) + dT * v1(:,time);
    theta1(:,time+1) = atan(v1(2,time+1)./v1(1,time+1));
    
    v2(:,time+1) = v2(:,time) + dT * u2(:,time);
    p2(:,time+1) = p2(:,time) + dT * v2(:,time);
    theta2(:,time+1) = atan(v2(2,time+1)./v2(1,time+1));
    
    v3(:,time+1) = v3(:,time) + dT * u3(:,time);
    p3(:,time+1) = p3(:,time) + dT * v3(:,time);
    theta3(:,time+1) = atan(v3(2,time+1)./v3(1,time+1));
    
    v4(:,time+1) = v4(:,time) + dT * u4(:,time);
    p4(:,time+1) = p4(:,time) + dT * v4(:,time);
    theta4(:,time+1) = atan(v4(2,time+1)./v4(1,time+1));
   
    % 记录时间信息
    t(1,time+1) = t(1,time) + dT;
    
    % 辅助绘图信息
    z1(1,time+1) = z1(1,time) + dT * 0;
    if zA(1,time)<10
        uA = 30;
    else
        uA = 0;
    end
    zA(1,time+1) = zA(1,time) + dT * uA;

    
end

if huitu6 == 1
    figure(6)
    plot(p0(1,:),p0(2,:),'-','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    plot(p1(1,:),p1(2,:),'-+','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    plot(p2(1,:),p2(2,:),'-x','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    plot(p3(1,:),p3(2,:),'-o','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    plot(p4(1,:),p4(2,:),'-d','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    axis equal
    rectangle('Position',[p0(1,times+1)-R,p0(2,times+1)-R,2*R,2*R],'linestyle','-.','Curvature',[1,1],'linewidth',1);
    rectangle('Position',[po(1,      1)-Rs,po(2,      1)-Rs,2*Rs,2*Rs],'linestyle','-.','Curvature',[1,1],'linewidth',1);
    rectangle('Position',[po(1,      1)-Rs2,po(2,      1)-Rs2,2*Rs2,2*Rs2],'linestyle','-','Curvature',[1,1],'linewidth',1);
    if strategyType == 1
        axis([-10,30, -10,30]);
    elseif strategyType == 2
        axis([-20,70, -10,60]);
    else
        axis([-15,55, -40,30]);
    end
    if avoidanceStatic == 1
    title('Obstacle Avoidance Control'); legend('p0','p1','p2','p3','p4');
    else
    title('No Obstacle Avoidance Control'); legend('p0','p1','p2','p3','p4');
    end
    xlabel('X axis'); ylabel('Y axis'); zlabel('Height'); grid on
    
end

if huitu5 == 1
    % 避障分析
    figure(5)

    subplot(2,4,1); % 轨迹图
%     plot(p0(1,:),p0(2,:),'-.','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    plot(p1(1,:),p1(2,:),'-+','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
%     plot(p2(1,:),p2(2,:),'-x','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
%     plot(p3(1,:),p3(2,:),'-o','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
%     plot(p4(1,:),p4(2,:),'-d','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    axis equal
    rectangle('Position',[p0(1,times+1)-R,p0(2,times+1)-R,2*R,2*R],'linestyle','-.','Curvature',[1,1],'linewidth',1);
    rectangle('Position',[po(1,      1)-Rs,po(2,      1)-Rs,2*Rs,2*Rs],'linestyle','-.','Curvature',[1,1],'linewidth',1);
    if strategyType == 1
        axis([-10,30, -10,30]);
    elseif strategyType == 2
        axis([-10,60, -10,60]);
    else
        axis([-15,55, -40,30]);
    end
    title('Trajectory'); legend('p0','p1','p2','p3','p4');
    xlabel('X axis'); ylabel('Y axis'); zlabel('Height'); grid on
    
    subplot(2,4,2); % 与障碍物位置差
    plot(t,p1(1,:)-po(1,1),'-+','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    plot(t,t-t+Rs,'-','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    title('Position - Time'); legend('p1X-O');
    xlabel('Time'); ylabel('Distance'); grid on
    
    subplot(2,4,3); % 斥力
    plot(t(1,1:length(t)-1),F1(1,:),'-+','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    title('Repulsive - Time'); legend('F1X');
    xlabel('Time'); ylabel('Repulsive'); grid on

    subplot(2,4,4); % 速度
    plot(t,v1(1,:),'-+','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    title('Velocity - Time'); legend('v1X');
    xlabel('Time'); ylabel('Velocity'); grid on
  
    subplot(2,4,5); % 输入
    plot(t(:,1:length(t)-1),u1(1,:),'-+','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    plot(t(1,1:length(t)-1),u1(2,:),'-x','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    title('Velocity - Time'); legend('u1X','u1Y');
    xlabel('Time'); ylabel('Input'); grid on
    
    subplot(2,4,6); % 与障碍物位置差
    plot(t,p1(2,:)-po(2,1),'-x','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    plot(t,t-t+Rs,'-','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    title('Position - Time'); legend('p1Y-O');
    xlabel('Time'); ylabel('Distance'); grid on
    
    subplot(2,4,7); % 斥力
    plot(t(1,1:length(t)-1),F1(2,:),'-x','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    title('Repulsive - Time'); legend('F1Y');
    xlabel('Time'); ylabel('Repulsive'); grid on

    subplot(2,4,8); % 速度
    plot(t,v1(2,:),'-x','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    title('Velocity - Time'); legend('v1Y');
    xlabel('Time'); ylabel('Velocity'); grid on
    
%     subplot(2,3,6); % 角度
%     theta1 = atan(v1(2,:)./v1(1,:));
% %     plot(t,v1(1,:),'-+','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
% %     plot(t,v1(2,:),'-x','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
%     plot(t,theta1,'-x','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
%     title('Velocity - Time'); legend('v1X','v1Y');
%     xlabel('Time'); ylabel('Velocity'); grid on
end

if huitu4 == 1
    figure(4)

    subplot(2,3,1); % 轨迹图
    plot(p0(1,:),p0(2,:),'-','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    plot(p1(1,:),p1(2,:),'-+','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    plot(p2(1,:),p2(2,:),'-x','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    plot(p3(1,:),p3(2,:),'-o','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    plot(p4(1,:),p4(2,:),'-d','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
%     axis equal
    rectangle('Position',[p0(1,times+1)-R,p0(2,times+1)-R,2*R,2*R],'linestyle','-.','Curvature',[1,1],'linewidth',1);
    if avoidanceStatic == 1
    rectangle('Position',[po(1,      1)-Rs,po(2,      1)-Rs,2*Rs,2*Rs],'linestyle','-.','Curvature',[1,1],'linewidth',1);
    rectangle('Position',[po(1,      1)-Rs2,po(2,      1)-Rs2,2*Rs2,2*Rs2],'linestyle','-','Curvature',[1,1],'linewidth',1);
    end
    if strategyType == 1
        axis([-10,30, -10*0.85,30*0.85]);
        title('(a)','position',[10,-10*0.85-5]);
        set(gca,'FontName','Times New Roman','Position',[0.04 0.575 0.28 0.40]);
    elseif strategyType == 2
        axis([-10,60, -10*0.85,60*0.85]);
        title('(a)','position',[25,-10*0.85-9]);
        set(gca,'FontName','Times New Roman','Position',[0.04 0.575 0.28 0.40]);
    else
        axis([-15,55, -40*0.85,30*0.85]);
        title('(a)','position',[20,-40*0.85-9]);
        set(gca,'FontName','Times New Roman','Position',[0.04 0.575 0.28 0.40]);
    end
%     title('Trajectory'); 
    legend('p0','p1','p2','p3','p4');
    xlabel('X axis'); ylabel('Y axis'); zlabel('Height'); grid on

    
    subplot(2,3,2); % 位置差
    p0A = sqrt((p0(1,:)-pA(1,:)).^2+(p0(2,:)-pA(2,:)).^2);
    p01 = sqrt((p0(1,:)-p1(1,:)).^2+(p0(2,:)-p1(2,:)).^2);
    p02 = sqrt((p0(1,:)-p2(1,:)).^2+(p0(2,:)-p2(2,:)).^2);
    p03 = sqrt((p0(1,:)-p3(1,:)).^2+(p0(2,:)-p3(2,:)).^2);
    p04 = sqrt((p0(1,:)-p4(1,:)).^2+(p0(2,:)-p4(2,:)).^2);
    plot(t,p0A,'-+','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,p01,'-x','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,p02,'-o','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,p03,'-d','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,p04,'-^','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    axis([0,tEnd, 0,30]); 
%     title('Distance - Time'); 
    legend('d0A','d01','d02','d03','d04');
    xlabel('Time'); ylabel('Distance'); grid on
    title('(b)','Position',[5,-4.5]);
    set(gca,'FontName','Times New Roman','Position',[0.37 0.575 0.28 0.40]);

    subplot(2,3,3); % 速度差
    v0A = sqrt(vA(1,:).^2+vA(2,:).^2) - v0;
    v01 = sqrt(v1(1,:).^2+v1(2,:).^2) - v0;
    v02 = sqrt(v2(1,:).^2+v2(2,:).^2) - v0;
    v03 = sqrt(v3(1,:).^2+v3(2,:).^2) - v0;
    v04 = sqrt(v4(1,:).^2+v4(2,:).^2) - v0;
    plot(t,v0A,'-+','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,v01,'-x','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,v02,'-o','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,v03,'-d','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,v04,'-^','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    axis([0,tEnd, -10,30]); 
%     title('Velocity Difference - Time'); 
    legend('v0A','v01','v02','v03','v04');
    xlabel('Time'); ylabel('Velocity Difference'); grid on
    title('(c)','position',[5,-16]);
    set(gca,'FontName','Times New Roman','Position',[0.70 0.575 0.28 0.40]);
    
        
    subplot(2,3,4); % 轨迹图
    plot3(p0(1,:),p0(2,:),z1,'-','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    plot3(pA(1,:),pA(2,:),zA,'-+','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    axis equal
    rectangle('Position',[p0(1,times+1)-R,p0(2,times+1)-R,2*R,2*R],'linestyle','-.','Curvature',[1,1],'linewidth',1);
    if strategyType == 1
        axis([-10,30, -10,30, 0,20]);
        title('(d)','position',[-10,-10,-9]);
    elseif strategyType == 2
        axis([-10,60, -10,60, 0,20]);
        title('(d)','position',[-10,-10,-23]);
    else
        axis([-15,55, -40,30, 0,20]);
        title('(d)','position',[-15,-40,-23]);
    end
%     title('Trajectory'); 
    legend('p0','pA');
    xlabel('X axis'); ylabel('Y axis'); zlabel('Height'); grid on
    
    set(gca,'FontName','Times New Roman','Position',[0.04 0.075 0.28 0.40]);
    
    subplot(2,3,5); % 位置差
    d12 = sqrt( (p1(1,:)-p2(1,:)).^2+(p1(2,:)-p2(2,:)).^2 );
    d13 = sqrt( (p1(1,:)-p3(1,:)).^2+(p1(2,:)-p3(2,:)).^2 );
    d14 = sqrt( (p1(1,:)-p4(1,:)).^2+(p1(2,:)-p4(2,:)).^2 );
    d23 = sqrt( (p2(1,:)-p3(1,:)).^2+(p2(2,:)-p3(2,:)).^2 );
    d24 = sqrt( (p2(1,:)-p4(1,:)).^2+(p2(2,:)-p4(2,:)).^2 );
    d34 = sqrt( (p3(1,:)-p4(1,:)).^2+(p3(2,:)-p4(2,:)).^2 );
    plot(t,d12,'-+','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,d13,'-x','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,d14,'-o','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,d23,'-d','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,d24,'-^','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,d34,'->','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    axis([0,tEnd, 0,30]); 
%     title('Distance - Time'); 
    legend('d12','d13','d14','d23','d24','d34');
    xlabel('Time'); ylabel('Distance'); grid on
    title('(e)','position',[5,-4.5]);
    set(gca,'FontName','Times New Roman','Position',[0.37 0.075 0.28 0.40]);
    
    subplot(2,3,6); % 速度差
    v12 = sqrt( (v1(1,:)-v2(1,:)).^2+(v1(2,:)-v2(2,:)).^2 );
    v13 = sqrt( (v1(1,:)-v3(1,:)).^2+(v1(2,:)-v3(2,:)).^2 );
    v14 = sqrt( (v1(1,:)-v4(1,:)).^2+(v1(2,:)-v4(2,:)).^2 );
    v23 = sqrt( (v2(1,:)-v3(1,:)).^2+(v2(2,:)-v3(2,:)).^2 );
    v24 = sqrt( (v2(1,:)-v4(1,:)).^2+(v2(2,:)-v4(2,:)).^2 );
    v34 = sqrt( (v3(1,:)-v4(1,:)).^2+(v3(2,:)-v4(2,:)).^2 );
    plot(t,v12,'-+','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,v13,'-x','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,v14,'-o','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,v23,'-d','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,v24,'-^','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    plot(t,v34,'->','linewidth',1,'MarkerIndices',1:60:length(t)); hold on 
    axis([0,tEnd, -10,30]); 
%     title('Velocity Difference - Time'); 
    legend('v12','v13','v14','v23','v24','v34');
    xlabel('Time'); ylabel('Velocity Difference'); grid on
    title('(f)','position',[5,-16]);
    set(gca,'FontName','Times New Roman','Position',[0.70 0.075 0.28 0.40]);
    
end

if huitu3 == 1
    figure(3)
    subplot(2,2,1); % 轨迹图
    plot3(p0(1,:),p0(2,:),z1,'-.','color','r','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    plot3(p1(1,:),p1(2,:),z1,'-+','color','g','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    plot3(p2(1,:),p2(2,:),z1,'-x','color','b','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    plot3(p3(1,:),p3(2,:),z1,'-o','color','m','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    plot3(p4(1,:),p4(2,:),z1,'-d','color','c','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    plot3(pA(1,:),pA(2,:),zA,'-^','color','y','linewidth',1,'MarkerIndices',1:6:length(t)); hold on
    axis equal
    rectangle('Position',[p0(1,times+1)-R,p0(2,times+1)-R,2*R,2*R],'linestyle','-.','Curvature',[1,1],'linewidth',1);
    axis([-10,50, -40,30, 0,20]); 
    title('Trajectory'); legend('p0','p1','p2','p3','p4','pA');
    xlabel('X axis'); ylabel('Y axis'); zlabel('Height'); grid on
    
    subplot(2,2,2); % 角度差
%     theta0 = atan(v0(2,:)./v0(1,:));
    theta1 = atan(v1(2,:)./v1(1,:));
    theta2 = atan(v2(2,:)./v2(1,:));
    theta3 = atan(v3(2,:)./v3(1,:));
    theta4 = atan(v4(2,:)./v4(1,:));
    theta01 = abs(theta0 - theta1);
    theta02 = abs(theta0 - theta2);
    theta03 = abs(theta0 - theta3);
    theta04 = abs(theta0 - theta4);
    plot(t,theta0,'-.','color','r','linewidth',1); hold on 
    plot(t,v0,'color','r','linewidth',1); hold on 
%     plot(t,theta1,'color','g','linewidth',1.5); hold on 
%     plot(t,theta2,'color','b','linewidth',1.5); hold on 
%     plot(t,theta3,'color','m','linewidth',1.5); hold on 
%     plot(t,theta4,'color','c','linewidth',1.5); hold on 
%     plot(t,theta01,'color','g','linewidth',1.5); hold on 
%     plot(t,theta02,'color','b','linewidth',1.5); hold on 
%     plot(t,theta03,'color','m','linewidth',1.5); hold on 
%     plot(t,theta04,'color','c','linewidth',1.5); hold on 
    axis([0,tEnd, -5,10]); 
    title("Target's velocity & angle  - Time"); legend('velocity','angle');
    xlabel('Time'); ylabel('Angle Difference'); grid on
    
    subplot(2,2,3); % 位置差
    p01 = sqrt((p0(1,:)-p1(1,:)).^2+(p0(2,:)-p1(2,:)).^2);
    p02 = sqrt((p0(1,:)-p2(1,:)).^2+(p0(2,:)-p2(2,:)).^2);
    p03 = sqrt((p0(1,:)-p3(1,:)).^2+(p0(2,:)-p3(2,:)).^2);
    p04 = sqrt((p0(1,:)-p4(1,:)).^2+(p0(2,:)-p4(2,:)).^2);
    p0A = sqrt((p0(1,:)-pA(1,:)).^2+(p0(2,:)-pA(2,:)).^2);
    plot(t,p01,'-+','color','g','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    plot(t,p02,'-x','color','b','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    plot(t,p03,'-o','color','m','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    plot(t,p04,'-d','color','c','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    plot(t,p0A,'-^','color','y','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    axis([0,tEnd, 0,20]); 
    title('Position Difference - Time'); legend('p01','p02','p03','p04','p0A');
    xlabel('Time'); ylabel('Position Difference'); grid on

    subplot(2,2,4); % 速度差
    v01 = sqrt(v1(1,:).^2+v1(2,:).^2) - v0;
    v02 = sqrt(v2(1,:).^2+v2(2,:).^2) - v0;
    v03 = sqrt(v3(1,:).^2+v3(2,:).^2) - v0;
    v04 = sqrt(v4(1,:).^2+v4(2,:).^2) - v0;
    v0A = sqrt(vA(1,:).^2+vA(2,:).^2) - v0;
    plot(t,v01,'-+','color','g','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    plot(t,v02,'-x','color','b','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    plot(t,v03,'-o','color','m','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    plot(t,v04,'-d','color','c','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    plot(t,v0A,'-^','color','y','linewidth',1,'MarkerIndices',1:6:length(t)); hold on 
    axis([0,tEnd, -10,30]); 
    title('Velocity Difference - Time'); legend('v01','v02','v03','v04','v0A');
    xlabel('Time'); ylabel('Velocity Difference'); grid on

end

if huitu2 == 1
    figure(2)
    subplot(); % 位置图
    plot(p0(1,:),p0(2,:),'linewidth',1.5); hold on
    plot(p1(1,:),p1(2,:),'linewidth',1.5); hold on
    plot(p2(1,:),p2(2,:),'linewidth',1.5); hold on
    plot(p3(1,:),p3(2,:),'linewidth',1.5); hold on
    plot(p4(1,:),p4(2,:),'linewidth',1.5); hold on
    axis([0,50, 0,50, 0,50]); 
    title('Trajectory'); legend('p0x','p1x','p2x','p3x','p4x');
    xlabel('X axis'); ylabel('Y axis'); grid on
end

if huitu == 1
    figure(1)
    % 绘图 Velocity Observer
    subplot(2,2,1)
    plot(p0(1,:),p0(2,:),'linewidth',1.5); hold on
    plot(pA(1,:),pA(2,:),'linewidth',1.5); hold on
    title('Position'); legend('Target位置', 'UAV位置'); 
    xlabel('X Position');ylabel('Y Position');zlabel('Height'); grid on;
    
    subplot(2,2,2)
    plot(t, p0(1,:)-pA(1,:),'linewidth',1.5); hold on
    plot(t, p0(2,:)-pA(2,:),'linewidth',1.5); hold on
    title('Position Difference - Time'); legend('x 位置差值', 'y 位置差值'); 
    xlabel('Time');ylabel('Position Difference');zlabel('Height'); grid on

    subplot(2,2,3)
    plot(t, vA(1,:),'linewidth',1.5); hold on
    plot(t, vA(2,:),'linewidth',1.5); hold on
    title('Velocity'); legend('X Velocity', 'Y Velocity');
    xlabel('Time');ylabel('Velocity');zlabel('Height'); grid on
    
    subplot(2,2,4)
    plot(t, v0(1,:)-vA(1,:),'linewidth',1.5); hold on
    plot(t, v0(2,:)-vA(2,:),'linewidth',1.5); hold on
    title('Velocity Difference - Time'); legend('x 速度差值', 'y 速度差值'); 
    xlabel('Time');ylabel('Velocity Difference');zlabel('Height'); grid on
end










