clear;
clc;

m = 3980;
x_os = [0 0 0 0 0 0]';
% xm_obs = [0 1300 180 10 0 0]';
xm_obs = [500 500 270 9 0 0
          3536 13000 180 4 0 0]';
xs_obs = [0 1800
         821 3800
         2536 5535]';

numM = size(xm_obs,2);%计算动态障碍物的数量
numS = size(xs_obs,2);%计算动态障碍物的数量

% 船舶状态空间里的航向改成数学坐标系的角度（弧度）

x_os(3) = computeAngle(x_os(3))*pi/180;

for i = 1:numM
    xm_obs(3,i) = computeAngle(xm_obs(3,i))*pi/180;
end

Kp = 0.1; Ku = 800;
rs = [200 300 150];
L = [100 100];
np = 24; % 预测时域
ns = 2; % 控制时域

X_OS(1,:) = [x_os(1,1) x_os(2,1)];
u_os(1,:) = sqrt(x_os(4,1)^2+x_os(5,1)^2);

pd_all(1,:) = [0 0];

%生成计划航迹

angle1 = 90*pi/180; angle2 = 45*pi/180; angle3 = 90*pi/180;
dis = 20;
for i = 1:650
    if ( i <= 150)
        pd_all(i+1,:) = [pd_all(i,1)+dis*cos(angle1) pd_all(i,2)+dis*sin(angle1)];
    elseif(150<i && i<=400)
        pd_all(i+1,:) = [pd_all(i,1)+dis*cos(angle2) pd_all(i,2)+dis*sin(angle2)];
    else
        pd_all(i+1,:) = [pd_all(i,1)+dis*cos(angle3) pd_all(i,2)+dis*sin(angle3)];
    end
end

X = [];   
Y = [];
N = [];

% 输入X Y N 限制

lb = [-6550 -645 -4*645];
ub = [13100 645 4*645];

pdd = [];

X_OS = [];
X_OS(1,:) = [x_os(1,1) x_os(2,1)];

t_fin = 700;
for t = 1:t_fin 
    
    % 在pd_all找到与当前时刻最近的一个点作为计算cost时的pd的起始点，若有多个最近点则取下标最大的点
    
    delta = [pd_all(:,1)-x_os(1,1) pd_all(:,2)-x_os(2,1)];
    dis = sum(delta.*delta,2);
    min_all = find(dis == min(dis));
    row = max(min_all);
    pd = pd_all(row:row+np-1,:);
    pdd(t,:) = pd_all(row,:);

    options = optimset( 'LargeScale', 'off', 'GradObj','off', 'GradConstr','off',...
    'DerivativeCheck', 'off', 'TolX', 1e-4,...
    'TolFun', 1e-4, 'TolCon', 1e-4, 'MaxFunEvals',5000,...
    'DiffMinChange',1e-4,'Algorithm','interior-point');
    
    miu = [];
    if numM > 0
        for i = 1:numM
            x_obs = xm_obs(:,i);
            miu(i) = computeMiu(x_os,x_obs);
        end
    end
    
    Miu(t) = max(miu);
    [x, y, exitflag] = fmincon(@(u)computeCost(u,Kp,Ku,pd,x_os,np,Miu(t)), rand(3,1), [], [], [], [], lb, ub, ...
        @(u)constraint(u,x_os,xs_obs,rs,xm_obs,np,L), options);

    row =size(X_OS,1);
    for i = 1:ns
        x_os = rk4(@model,x_os,x ,1);
        X_OS(row+i,:) = [x_os(1,1) x_os(2,1)];
        u_os(row+i) = sqrt(x_os(4,1)^2+x_os(5,1)^2);
    end
    if numM > 0
        for i = 1:numM
            xm(1,:,i) = [xm_obs(1,i) xm_obs(2,i)];
            xm_obs(:,i)= [  xm_obs(4,i)*cos(xm_obs(3,i))*ns+xm_obs(1,i)
                            xm_obs(4,i)*sin(xm_obs(3,i))*ns+xm_obs(2,i)
                            xm_obs(3,i)
                            xm_obs(4,i)
                            xm_obs(5,i)
                            xm_obs(6,i)                             ];
            xm(t+1,:,i) = [xm_obs(1,i) xm_obs(2,i)];
        end
    end
    

end

figure;

if numS > 0
    for i = 1:numS
        aplha=0:pi/40:2*pi;
        r=2;
        x=rs(i)*cos(aplha)+xs_obs(1,i);
        y=rs(i)*sin(aplha)+xs_obs(2,i);
        plot(x,y,'-');
        axis equal  
        hold on;
    end
end

if numM > 0
    for i = 1:numM
        plot(xm(:,1,i),xm(:,2,i),'o','linewidth',1.0);
        hold on;
    end
end

plot(X_OS(:,1),X_OS(:,2),'*','linewidth',1.0);
hold on;

plot(pd_all(:,1),pd_all(:,2),'--','linewidth',1.0);
hold on;