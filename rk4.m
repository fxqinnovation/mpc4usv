function xnext = rk4(f,x,u,h)
% RK4	Integrate a system of ordinary differential equations using
%	    Runge-Kutta's 4th-order method. The control input u can be constant
%	    over the sampling interval h or a time-varying function u = g(x,t).
%
% The output xnext = x(k+1) is:
%
%    xnext = rk4('f',x,'g',h,t)     nonautonomous systems 
%    xnext = rk4('f',x,u,h)         autonomous systems (with constant u)
%
% where
%
%    f     - external function: 
%            dx/dt(k) = f(x(k),u(k),t(k))  nonautonomous systems
%            dx/dt(k) = f(x(k),u(k))       autonomous systems
%
%    x     - x(k)
%    u     - external function: u(k) = g(x(k),t(k))   nonautonomous systems
%            u(k) = constant over the sample time h   autonomous systems
%    h     - step size
%    t     - time t(k) - ONLY NEEDED FOR NONAUTONOMOUS SYSTEMS
%
% Ex 1:   function u = g(x,t)
%           u = -2*x + cos(t);
%
%         function dx = f(x,u,t)
%           dx = -x^2 + u + cos(t);     
%
%         ===>  xnext = rk4('f',x,'g',h,t) 
%
% Ex 2:   u = constant
%
%         function dx = f(x,u), 
%           dx = -x^2 + u;
%
%         ===>  xnext = rk4('f',x,u,h)
%
% Author:    Thor I. Fossen
% Date:      2001-07-14
% Revisions: 2007-11-23 Christian Holden - update for time-varying systems
% ________________________________________________________________
%
% MSS GNC is a Matlab toolbox for guidance, navigation and control.
% The toolbox is part of the Marine Systems Simulator (MSS).
%
% Copyright (C) 2008 Thor I. Fossen and Tristan Perez
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
% GNU General Public License formore details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
% 
% E-mail: contact@marinecontrol.org
% URL:    <http://www.marinecontrol.org>

xo = x;
k1 = h*feval(f,xo,u);
x  = xo+0.5*k1;
k2 = h*feval(f,x,u);
x  = xo+0.5*k2;
k3 = h*feval(f,x,u);
x  = xo+k3;
k4 = h*feval(f,x,u);
xnext = xo + (k1+2*(k2+k3)+k4)/6;