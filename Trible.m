% 二阶仿真
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
dT     = 0.1;
times  = (tEnd-tBegin)/dT;
t(1,1) = 0;

% 其他关键参数
K = 2;          % 速度观测器
kappa = 0.5;
alpha = 0.10;      % 一致性参数
beta  = 0.10;
gamma = 1;      % 追捕参数
gasp  = 1;
R = 10;         % 追捕半径

strategyType = 3;

huitu  = 0;     % 速度观测器
huitu2 = 0;
huitu3 = 1;

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

    % UGV 的轨迹
    u1 = alpha * ( (p2(:,time)-p1(:,time))+(p3(:,time)-p1(:,time))+(p4(:,time)-p1(:,time)) )...
       + beta  * ( (v2(:,time)-v1(:,time))+(v3(:,time)-v1(:,time))+(v4(:,time)-v1(:,time)) )...
       + gamma * (  p0(:,time)-p1(:,time) + R * [cos(theta0(1,time) + (1-1)*0.5*pi);sin(theta0(1,time) + (1-1)*0.5*pi)])...
       + gasp    * (  vA(:,time+1)        -v1(:,time));
   
    u2 = alpha * ( (p1(:,time)-p2(:,time))+(p3(:,time)-p2(:,time))+(p4(:,time)-p2(:,time)) )...
       + beta  * ( (v1(:,time)-v2(:,time))+(v3(:,time)-v2(:,time))+(v4(:,time)-v2(:,time)) )...
       + gamma * (  p0(:,time)-p2(:,time) + R * [cos(theta0(1,time) + (2-1)*0.5*pi);sin(theta0(1,time) + (2-1)*0.5*pi)])...
       + gasp    * (  vA(:,time+1)        -v2(:,time));
   
    u3 = alpha * ( (p1(:,time)-p3(:,time))+(p2(:,time)-p3(:,time))+(p4(:,time)-p3(:,time)) )...
       + beta  * ( (v1(:,time)-v3(:,time))+(v2(:,time)-v3(:,time))+(v4(:,time)-v3(:,time)) )...
       + gamma * (  p0(:,time)-p3(:,time) + R * [cos(theta0(1,time) + (3-1)*0.5*pi);sin(theta0(1,time) + (3-1)*0.5*pi)])...
       + 1    * (  vA(:,time+1)        -v3(:,time));
    
    u4 = alpha * ( (p1(:,time)-p4(:,time))+(p2(:,time)-p4(:,time))+(p3(:,time)-p4(:,time)) )...
       + beta  * ( (v1(:,time)-v4(:,time))+(v2(:,time)-v4(:,time))+(v3(:,time)-v4(:,time)) )...
       + gamma * (  p0(:,time)-p4(:,time) + R * [cos(theta0(1,time) + (4-1)*0.5*pi);sin(theta0(1,time) + (4-1)*0.5*pi)])...
       + gasp    * (  vA(:,time+1)        -v4(:,time));

    v1(:,time+1) = v1(:,time) + dT * u1;
    p1(:,time+1) = p1(:,time) + dT * v1(:,time);
    theta1(:,time+1) = atan(v1(2,time+1)./v1(1,time+1));
    
    v2(:,time+1) = v2(:,time) + dT * u2;
    p2(:,time+1) = p2(:,time) + dT * v2(:,time);
    theta2(:,time+1) = atan(v2(2,time+1)./v2(1,time+1));
    
    v3(:,time+1) = v3(:,time) + dT * u3;
    p3(:,time+1) = p3(:,time) + dT * v3(:,time);
    theta3(:,time+1) = atan(v3(2,time+1)./v3(1,time+1));
    
    v4(:,time+1) = v4(:,time) + dT * u4;
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
    title('Trajectry'); legend('p0','p1','p2','p3','p4','pA');
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
    title('Trajectry'); legend('p0x','p1x','p2x','p3x','p4x');
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


% function c1,c2,c3,c4 = Consensus(p1,p2,p3,p4,v1,v2,v3,v4)
%     % 一致性函数
%     c1 = alpha * ( (p2(:,time)-p1(:,time))+(p3(:,time)-p1(:,time))+(p4(:,time)-p1(:,time)) )...
%        + beta  * ( (v2(:,time)-v1(:,time))+(v3(:,time)-v1(:,time))+(v4(:,time)-v1(:,time)) );
%    
%     c2 = alpha * ( (p1(:,time)-p2(:,time))+(p3(:,time)-p2(:,time))+(p4(:,time)-p2(:,time)) )...
%        + beta  * ( (v1(:,time)-v2(:,time))+(v3(:,time)-v2(:,time))+(v4(:,time)-v2(:,time)) );
%    
%     c3 = alpha * ( (p1(:,time)-p3(:,time))+(p2(:,time)-p3(:,time))+(p4(:,time)-p3(:,time)) )...
%        + beta  * ( (v1(:,time)-v3(:,time))+(v2(:,time)-v3(:,time))+(v4(:,time)-v3(:,time)) );
%    
%     c4 = alpha * ( (p1(:,time)-p4(:,time))+(p2(:,time)-p4(:,time))+(p3(:,time)-p4(:,time)) )...
%        + beta  * ( (v1(:,time)-v4(:,time))+(v2(:,time)-v4(:,time))+(v3(:,time)-v4(:,time)) );
% end







