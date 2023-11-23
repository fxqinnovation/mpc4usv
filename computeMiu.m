function miu = computeMiu(x_os,xm_obs)
 
% miu是一个0 1变量，当miu=1时符合rule14或15

d_close = 1852;
distance = sqrt((x_os(1)-xm_obs(1))^2+(x_os(2)-xm_obs(2))^2);

vel_os = [sqrt(x_os(4)^2+x_os(5)^2)*cos(x_os(3)) sqrt(x_os(4)^2+x_os(5)^2)*sin(x_os(3))];
vel_obs = [sqrt(xm_obs(4)^2+xm_obs(5)^2)*cos(xm_obs(3)) sqrt(xm_obs(4)^2+xm_obs(5)^2)*sin(xm_obs(3))];
L = [xm_obs(1)-x_os(1) xm_obs(2)-x_os(2)];

module_vel_os = sqrt(vel_os(1)^2+vel_os(2)^2);
module_vel_obs = sqrt(vel_obs(1)^2+vel_obs(2)^2);
module_L = sqrt(L(1)^2+L(2)^2);

port = computePort(x_os,xm_obs);

if distance <= d_close
    
    if vel_os*L' > cos(6*pi/180)*module_vel_os*module_L
        
        if vel_os*vel_obs' < cos(112.5*pi/180)*module_vel_os*module_vel_obs %对遇方位
            miu = 1; %rule14
        else
            miu = 0;
        end
        
    elseif vel_os*L' > cos(112.5*pi/180)*module_vel_os*module_L  && port == 0
        
        if vel_os*vel_obs' < cos(6*pi/180)*module_vel_os*module_vel_obs && vel_os*vel_obs' > cos(112.5*pi/180)*module_vel_os*module_vel_obs
            miu = 1; %rule15
        else
            miu = 0;
        end
        
    else
        miu = 0;
    end
    
else
    miu = 0;
end

end