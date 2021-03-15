% 速度观测器
clear
clc

% 目标运动轨迹
target1 = 0;
target2 = 0;
target3 = 0;
target4 = 0;

% 目标逃逸方案1：静止不动
if target1 == 1
    v0 = 0;
    u0 = 0;
end
% 目标逃逸方案2：匀速运动
if target2 == 1
    v0 = 1;
    u0 = 0;
end
% 目标逃逸方案3
if target3 == 1
end
% 目标逃逸方案4
if target4 == 1
end

p0(:,1) = [10;5];  
% theta0 = 0;
% v0(:,1) = 0 * [cos(theta0); sin(theta0);];
v0(:,1) = [0,0]';

% UAV 运动轨迹
pA(:,1) = [0;0]; vA(:,1) = [0*cos(0);0*sin(0)]; thetaA(1,1) = 0;

% 时间参数
tBegin = 0;
tEnd   = 30;
dT     = 0.01;
times  = (tEnd-tBegin)/dT;
t(1,1) = 0;

% 其他关键参数
K = 1;          % 速度观测器
kappa = 0.0;

huitu = 1;
huitu2= 0;

% 绘制 UGV 三维图辅助信息
z1(1,1) = 0;
zA(1,1) = 0;

for time = 1:times
    % 目标的速度
    if t(1,time)>=0 && t(1,time)<1      % 45度斜向上
        u0 = [3,3]';
    elseif t(1,time)>=1 && t(1,time)<2  % 减速
        u0 = [0,-3]';   
    elseif t(1,time)>=2 && t(1,time)<4 % 水平向右
        u0 = [0,0]';
    elseif t(1,time)>=4 && t(1,time)<5 % 减速
        u0 = [-3,3]';   
    elseif t(1,time)>=5 && t(1,time)<7 % 垂直向上
        u0 = [0,0]';
    elseif t(1,time)>=7 && t(1,time)<8 % 
        u0 = [-3,-3]';
    elseif t(1,time)>=8 && t(1,time)<10
        u0 = [0,0]';
    elseif t(1,time)>=10 && t(1,time)<11
        u0 = [3,-2.5]';
    elseif t(1,time)>=11 && t(1,time)<13
        u0 = [0,0]';
    elseif t(1,time)>=13 && t(1,time)<14
        u0 = [0,2.5]';    
    elseif t(1,time)>=14 && t(1,time)<15
        u0 = [3,0]';
    elseif t(1,time)>=16 && t(1,time)<23
        u0 = [-3*sin(t(1,time)-16),3*cos(t(1,time)-16)]';
    else
        u0=0;
    end
    % 目标的轨迹
    v0(:,time+1) = v0(:,time) + dT .* u0;
    p0(:,time+1) = p0(:,time) + dT * v0(:,time+1);
    
    % UAV 的轨迹，需要传递观测到的速度
    vA(:,time+1) = K * (p0(:,time)-pA(:,time)) + kappa * vA(:,time);
    pA(:,time+1) = pA(:,time) + dT * vA(:,time+1);
    % 估算值
    vE(:,time+1) = vA(:,time+1) - K * (p0(:,time)-pA(:,time));
    
    % 记录时间信息
    t(1,time+1) = t(1,time) + dT;
    
    % 辅助绘图信息
    z1(1,time+1) = z1(1,time) + dT * 0;
    if zA(1,time)<15
        uA = 30;
    else
        uA = 0;
    end
    zA(1,time+1) = zA(1,time) + dT * uA;
    
end

if huitu == 1
    figure(1)
    % 绘图 Velocity Observer

    subplot(2,2,1)
    plot3(p0(1,:),p0(2,:),z1,'linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    plot3(pA(1,:),pA(2,:),zA,'-+','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    axis([0,40, 0,30, 0,30]);
%     title('Trajectory'); 
    title('(a)','position',[0,0,-8]);
    legend("target trajectory", "UAV trajectory"); 
    xlabel('X Position');ylabel('Y Position');zlabel('Height'); grid on;
    set(gca,'FontName','Times New Roman','position', [0.05 0.575 0.40 0.40]);
    
    subplot(2,2,2)
%     plot(pA(1,:),pA(2,:),'linewidth',1.5,'color','g'); hold on
%     plot(t, p0(1,:)-pA(1,:),'linewidth',1.5,'color','r'); hold on
%     plot(t, p0(2,:)-pA(2,:),'linewidth',1.5,'color','g'); hold on
    p0A = sqrt( (p0(1,:)-pA(1,:)).^2 + (p0(2,:)-pA(2,:)).^2 );
    plot(t, p0A,'linewidth',1); hold on
    axis([0,tEnd, -10,40]);
%     title('Position Difference - Time'); %legend('x 轴位置差值', 'y 轴位置差值'); 
    title('(b)','position',[15,-18]);
    xlabel('Time');ylabel('Position Difference');zlabel('Height'); grid on;
    set(gca,'FontName','Times New Roman','position', [0.55 0.575 0.40 0.40]);

    subplot(2,2,3)
    v0V = sqrt( (v0(1,:)).^2 + (v0(2,:)).^2 );
    vAV = sqrt( (vA(1,:)).^2 + (vA(2,:)).^2 );
    plot(t, v0V,'linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    plot(t, vAV,'-+','linewidth',1,'MarkerIndices',1:60:length(t)); hold on
    axis([0,tEnd, -10,40]);
%     title('Velocity'); 
    title('(c)','position',[15,-18]);
    legend("target velocity", "UAV velocity");
    xlabel('Time');ylabel('Velocity');zlabel('Height'); grid on;
    set(gca,'FontName','Times New Roman','position', [0.05 0.075 0.40 0.40]);
    
    subplot(2,2,4)
    v0V = sqrt( (v0(1,:)).^2 + (v0(2,:)).^2 );
    vAV = sqrt( (vA(1,:)).^2 + (vA(2,:)).^2 );
    plot(t, vAV-v0V,'linewidth',1); hold on
%     plot(t, v0(1,:)-vA(1,:),'linewidth',1.5,'color','r'); hold on
%     plot(t, v0(2,:)-vA(2,:),'linewidth',1.5,'color','g'); hold on
%     plot(t, vA(1,:),'linewidth',1.5,'color','b'); hold on
%     plot(t, vA(2,:),'linewidth',1.5,'color','c'); hold on
    axis([0,tEnd, -10,40]);
%     title('Velocity Difference - Time'); 
    title('(d)','position',[15,-18]);
    %legend('UAV X速度', 'UAV Y速度'); 
    xlabel('Time');ylabel('Velocity Difference');zlabel('Height'); grid on;
    set(gca,'FontName','Times New Roman','Position',[0.55 0.075 0.40 0.40]);
    
end

if huitu2 == 1
    figure(2)
    % 绘图 Velocity Observer
    subplot(2,2,1)
    plot(p0(1,:),p0(2,:),'linewidth',1.5,'color','r'); hold on
    title('Position'); legend('Target位置', 'UAV位置'); 
    xlabel('X Position');ylabel('Y Position');zlabel('Height'); grid on;
    
    subplot(2,2,2)
    plot(pA(1,:),pA(2,:),'linewidth',1.5,'color','g'); hold on
    title('Position'); legend('Target位置', 'UAV位置'); 
    xlabel('X Position');ylabel('Y Position');zlabel('Height'); grid on;

    subplot(2,2,3)
    v0V = sqrt( (v0(1,:)).^2 + (v0(2,:)).^2 );
    plot(t, v0V,'linewidth',1.5,'color','r'); hold on
    title('Velocity'); legend('目标速度', 'UAV 速度');
    xlabel('Time');ylabel('Velocity');zlabel('Height'); grid on
    
    subplot(2,2,4)
    vAV = sqrt( (vA(1,:)).^2 + (vA(2,:)).^2 );
    plot(t, vAV,'linewidth',1.5,'color','g'); hold on
    title('Velocity'); legend('目标速度', 'UAV 速度');
    xlabel('Time');ylabel('Velocity');zlabel('Height'); grid on
end

% pause(0.5)
% end