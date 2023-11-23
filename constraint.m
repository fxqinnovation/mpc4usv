 function [h,g] = constraint(u,x_os,xs_obs,rs,xm_obs,np,L)

numS = size(xs_obs,2);%计算静态障碍物的数量
numM = size(xm_obs,2);%计算动态障碍物的数量
    
for i = 1:np
    
    h = 1;%限定四阶龙格库塔的时间步长
    
    if (i==1)
        xnp_os(:,i) = rk4(@model,x_os,u,h);
        p_os(i,1) = xnp_os(1,i);
        p_os(i,2) = xnp_os(2,i);
    else
        xnp_os(:,i) = rk4(@model,xnp_os(:,i-1),u,h);
        p_os(i,1) = xnp_os(1,i);
        p_os(i,2) = xnp_os(2,i);
    end
    
    if numS == 0
        
        if numM == 0
            hs = []; hm = [];
        else
            hs = [];
            for m = 1:numM
                pm_obs(i,1,m) = xm_obs(4,m)*cos(xm_obs(3,m))*i+xm_obs(1,m);
                pm_obs(i,2,m) = xm_obs(4,m)*sin(xm_obs(3,m))*i+xm_obs(2,m); 
                hm(m,i) = 16*L(m)^2-((p_os(i,1)-pm_obs(i,1,m))*cos(xm_obs(3,m))+(p_os(i,2)-pm_obs(i,2,m))*sin(xm_obs(3,m)))^2 ...
                        -((pm_obs(i,1,m)-p_os(i,1))*sin(xm_obs(3,m))+(p_os(i,2)-pm_obs(i,2,m))*cos(xm_obs(3,m)))^2*6.25;
            end
        end
        
    else
        if numM == 0
            
            hm = [];
            for s = 1:numS
                ps_obs(i,1,s) = xs_obs(1,s);
                ps_obs(i,2,s) = xs_obs(2,s);
                hs(s,i) = rs(s)^2-((p_os(i,1)-ps_obs(i,1,s))^2+(p_os(i,2)-ps_obs(i,2,s))^2);
            end
            
        else
            
            for s = 1:numS
                ps_obs(i,1,s) = xs_obs(1,s);
                ps_obs(i,2,s) = xs_obs(2,s);
                hs(s,i) = rs(s)^2-((p_os(i,1)-ps_obs(i,1,s))^2+(p_os(i,2)-ps_obs(i,2,s))^2);
            end
            for m = 1:numM
                pm_obs(i,1,m) = xm_obs(4,m)*cos(xm_obs(3,m))*i+xm_obs(1,m);
                pm_obs(i,2,m) = xm_obs(4,m)*sin(xm_obs(3,m))*i+xm_obs(2,m); 
                hm(m,i) = 16*L(m)^2-((p_os(i,1)-pm_obs(i,1,m))*cos(xm_obs(3,m))+(p_os(i,2)-pm_obs(i,2,m))*sin(xm_obs(3,m)))^2 ...
                        -((pm_obs(i,1,m)-p_os(i,1))*sin(xm_obs(3,m))+(p_os(i,2)-pm_obs(i,2,m))*cos(xm_obs(3,m)))^2*6.25;
            end
            
        end
        
    end

end

h = [hs;hm];%不等式约束

g = 4*u(2)-u(3);%等式约束
end