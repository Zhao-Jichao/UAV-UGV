% 逃跑策略，共有四种
clear
clc

% 模型初始化变量及参数
p0(:,1) = [10,5]';
theta0   = 1.0517;
v0(1,1)  = 4.0315;


% 时间参数
tBegin = 0;
tEnd   = 10;
dT     = 0.2;
times  = (tEnd-tBegin)/dT;
t(1,1) = 0;

% 策略选择
strategyType = 2;
huitu = 1;


for time = 1:times
    if strategyType == 1
        % 1. 静止不动
        theta0   = 1.0517;
        v0(1,1)  = 0;
        u0       = 0;
    end

    if strategyType == 2 && t(1,time)<5
        % 2. 匀速直线运动
        theta0   = 1.0517;
        v0(1,1)  = 5;
        u0       = 1;
    end

    if strategyType == 3
        % 3. 最近追捕者的速度方向
        theta0   = 1.0517;
        v0(1,1)  = 5;
        u0       = 0;
    end

    if strategyType == 4
        % 4. 所有追捕者的标准化速度矢量和
        theta0   = 1.0517;
        v0(1,1)  = 5;
        u0       = 1;
    end

    % 记录目标轨迹
%     v0(1,time+1) = v0(1,time) + dT * u0;
    p0(1,time+1) = p0(1,time) + dT * v0 * cos(theta0);
    p0(2,time+1) = p0(2,time) + dT * v0 * sin(theta0);
    
    % 记录时间
    t(1, time+1) = t(1,time) + dT;
    
end

if huitu == 1
    % 绘制
    figure(1)
    plot(p0(1,:),p0(2,:),'>','color','r'); hold on
    legend('target 0');
    xlabel('X axis');
    ylabel('Y axis');
    axis([0,50, 0,50]); 
    axis equal;
    title('Fixed direction');
end

