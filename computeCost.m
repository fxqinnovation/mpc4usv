function Cost = computeCost(u,Kp,Ku,pd,x_os,np,Miu)

% x_os = [x y psi u v r]'

r = []; v = [];
% r(1) = x_os(3,1);
for i = 1:np
    h = 1;%限定四阶龙格库塔的时间步长
    if (i==1)
        xnp_os(:,i) = rk4(@model,x_os,u,h);
        v(i) = sqrt(xnp_os(4,i)^2+xnp_os(5,i)^2);
        p(i,1) = xnp_os(1,i);
        p(i,2) = xnp_os(2,i);
    else
        xnp_os(:,i) = rk4(@model,xnp_os(:,i-1),u,h);
        v(i) = sqrt(xnp_os(4,i)^2+xnp_os(5,i)^2);
        p(i,1) = xnp_os(1,i);
        p(i,2) = xnp_os(2,i);
    end
end

port = computePort(x_os,xnp_os(:,1));

cost = [];

for k = 1:np 
    
    x(k) = (p(k,1)-pd(k,1))^2+(p(k,2)-pd(k,2))^2;
    cost(k) =  Kp*x(k)+Ku*abs(v(k)-8);
    
end

Cost = sum(cost)*(Miu*port+1)^4;

end