% Author: Boyang Li
% The Hong Kong Polytechnic University
% email: boyang.li@connect.polyu.edu.hk
% Website: https://boyangli.com
% May 2018;

function state_dot = tailsitterDyna2(state,control,p)
% dx = tailsitterDyna(state,control,p)
%
% This function calculate derivative of the uav at
% given time, state, and control input
%
% INPUTS:
%   state = [3, n] = [xd;zd;theta]
%   u = [2, n] = [ft_cmd;theta_cmd]
%   p = parameter struct
%       .g = gravity
%       .mass = UAV mass
%       .rho = density
%       .tau = time constanct
%       .k = gain
% OUTPUTS:
%   state_dot = dx/dt = time derivative of state
%

% uppack
xd = state(1,:);
zd = state(2,:);
theta = state(3,:);
ft_cmd = control(1,:);
theta_cmd = control(2,:);
% Ft = thro_to_force(thro_cmd);

% aerodynamics //control surface?
% Cl0 = 0;
% Cla = 4.0;
% Cda = 0.5;
% aoa = atan2(u,-w);
% aoa = theta + 0.5*pi;
% AS = sqrt(u^2+w^2);
% AS = sqrt(xd.^2 + zd.^2);
% AS = xd;
% Cl = Cl0 + aoa .* Cla;
% Cd = 0.2 + aoa .* Cda;
% Cl = sin(2*aoa);
% Cd = 0.1 + 1.7 .* sin(aoa).^2;

% accurate aero params
% get aoa and as
aoa = theta + pi/2 + atan(zd/xd);
AS = sqrt(xd.^2+zd.^2);

% search Cl and Cd
% Cl = interp1(p.aoa,p.cl,aoa*180/pi);
% Cd = interp1(p.aoa,p.cd,aoa*180/pi);
Cl = ppval(p.pp_cl,aoa*180/pi);
Cd = ppval(p.pp_cd,aoa*180/pi);

% Aero force, wind axis
Fl = 0.5 * p.rho * AS.^2 * p.S .* Cl;
Fd = 0.5 * p.rho * AS.^2 * p.S .* Cd;
Fxb = -Fl.*cos(aoa)-Fd.*sin(aoa);
Fzb = -Fl.*sin(aoa)+Fd.*cos(aoa);
Fxi =  Fxb.*cos(theta)+Fzb.*sin(theta);
Fzi = -Fxb.*sin(theta)+Fzb.*cos(theta);

% derivatave
xdd = -ft_cmd .* sin(theta) ./ p.mass + Fxi;%-Fd
zdd = p.g - ft_cmd .* cos(theta)./ p.mass  + Fzi;%-Fl
thetad = (1/p.Ts) .* (theta_cmd - theta);
state_dot = [xdd; zdd; thetad];
end

% function ft = thro_to_force(u)
% if (u > 1)
%     u = 1;
% elseif (u<0)
%     u = 0;
% end
% ft_single = -10.861*u^3+21.803*u^2-2.0175*u+0.5661875;
% ft = ft_single * 4;
% end
