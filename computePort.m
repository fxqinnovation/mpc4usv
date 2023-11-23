function port = computePort(x_os,xm_obs)

% port是一个0 1变量，当port=1时xm_obs在x_os的左边

vel_os = [sqrt(x_os(4)^2+x_os(5)^2)*cos(x_os(3)) sqrt(x_os(4)^2+x_os(5)^2)*sin(x_os(3))];
L = [xm_obs(1)-x_os(1) xm_obs(2)-x_os(2)];

% 计算本船速度的右边的正交向量

ort_vel_os = null(vel_os);
if x_os(3) == 0
    
    if ort_vel_os(2) < 0
        ort_vel_os = ort_vel_os;
    else
        ort_vel_os = -ort_vel_os;
    end
    
elseif 0<x_os(3) && x_os(3)<pi/2
    
    if vel_os(1)*ort_vel_os(1) > 0
        ort_vel_os = ort_vel_os;
    else
        ort_vel_os = -ort_vel_os;
    end 
    
elseif x_os(3)==pi/2
    
    if ort_vel_os(1) > 0
        ort_vel_os = ort_vel_os;
    else
        ort_vel_os = -ort_vel_os;
    end
    
elseif pi/2<x_os(3) && x_os(3)<pi
    
    if vel_os(1)*ort_vel_os(1) < 0
        ort_vel_os = ort_vel_os;
    else
        ort_vel_os = -ort_vel_os;
    end 
    
elseif x_os(3) == pi
    
    if ort_vel_os(2) > 0
        ort_vel_os = ort_vel_os;
    else
        ort_vel_os = -ort_vel_os;
    end
    
elseif pi<x_os(3) && x_os(3)<pi*3/2
    
    if vel_os(1)*ort_vel_os(1) > 0
        ort_vel_os = ort_vel_os;
    else
        ort_vel_os = -ort_vel_os;
    end 
    
elseif x_os(3)==pi*3/2
    
    if ort_vel_os(1) < 0
        ort_vel_os = ort_vel_os;
    else
        ort_vel_os = -ort_vel_os;
    end
    
elseif pi/2<x_os(3) && x_os(3)<pi
    
    if vel_os(1)*ort_vel_os(1) < 0
        ort_vel_os = ort_vel_os;
    else
        ort_vel_os = -ort_vel_os;
    end 
    
elseif x_os(3) == 2*pi
    
    if ort_vel_os(2) < 0
        ort_vel_os = ort_vel_os;
    else
        ort_vel_os = -ort_vel_os;
    end
    
end

% 计算L与ort_vel_os的夹角

ang = acos(L*ort_vel_os/(sqrt(sum(L.*L))*sqrt(sum(ort_vel_os.*ort_vel_os))));

 if ang<= pi/2
     port = 0; %右边
 else
     port = 1;  %左边
 end
 
end