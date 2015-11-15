function xdot = msfartoystyring(x,u, curr)
% xdot = msfartoystyring(x,u,curr) returns the time derivative of the state
% vector: x = [ u v r x y psi]' for the MS Fartoystyring, where:
% u   = surge velocity, must be positive (m/s) - design speed u = 8.23 m/s
% v   = sway velocity                    (m/s)
% r   = yaw velocity                     (rad/s)
% x   = position in x-direction          (m)
% y   = position in y-direction          (m)
% psi = yaw angle                        (rad)
%        
% 
% The input vector is :
% u      = [ delta_c  n_c h]',  where
% delta_c = commanded rudder angle   (rad)
% n_c     = commanded shaft velocity (rad/s) - nominal propeller 80 rpm
%
% curr    = current on/off                    (0 or 1)
%
% Author:    Christian Holden
% Date:      2008-02-20
% Revisions: 2009-02-19, C. Holden: Changed current model, plus minor
%                        changes
%            2010-02-16, C. Holden: Updated current model, changed current
%                        direction
%            2010-03-03, C. Holden: Serious bug fix in current model

% Check of input and state dimensions
if (length(x)  ~= 6),error('x-vector must have dimension 6 !');end
if (length(u) ~= 2),error('u-vector must have dimension 3 !');end

% Normalization variables
L   =  304.8;          % length of ship (m)
g   =  9.81;            % acceleration of gravity (m/s^2)

% Dimensional states and input
delta_c = u(1); 
n_c     = u(2);

u     = x(1);    
v     = x(2); 
r     = x(3);
psi   = x(6); 

% Parameters, hydrodynamic derivatives and main dimensions
delta_max  = 25*pi/180;       % max rudder angle      (rad)
%Ddelta_max = 2.33;     % max rudder derivative (deg/s)
n_max      = 85*2*pi/60;       % max shaft velocity    (rad/s)

t   =  0.22;
Tm  =  50;
T   =  18.46;

cun =  0.605;  
cnn =  38.2;

Tuu = -0.00695;
Tun = -0.00063;
Tnn =  0.0000354;

m11 =  1.050;          % 1 - Xudot
m22 =  2.020;          % 1 - Yvdot
m33 =  0.1232;         % kz^2 - Nrdot

d11 =  2.020;          % 1 + Xvr
d22 = -0.752;          % Yur - 1
d33 = -0.231;          % Nur - xG 

                    YT     =  0.04;   NT      = -0.02;
Xuu    = -0.0377;   Yvv    = -2.400;  Nvr     = -0.300;
Xvv    =  0.3;      Yuv    = -1.205;  Nuv     = -0.451;   
Xccdd  = -0.093;    Yccd   =  0.208;  Nccd    = -0.098;
Xccbd  =  0.152;    Yccbbd = -2.16;   Nccbbd  =  0.688;
Xudot  =  1-m11;    Yvdot  =  1-m22;  

% Rudder saturation
if abs(delta_c) >= delta_max,
   delta = sign(delta_c)*delta_max;
else
    delta = delta_c;
end

% Shaft saturation
if abs(n_c) >= n_max,
   n = sign(n_c)*n_max;
else
   n = n_c;
end
n = n/(2*pi);

% Current (new)
Fxc=0;
Fyc=0;
ur = u;
vr = v;
if curr
    thc=-90*pi/180; %Current direction (from east)
    
    Rc=[cos(thc) -sin(thc); sin(thc) cos(thc)];
    Rt=[cos(psi) sin(psi); -sin(psi) cos(psi)];
    vcn=[.9; 0];
    vc=Rt*Rc*vcn;
    vcd=-[0 -r; r 0]*Rt*Rc*vcn;
    
    ur=u-vc(1);
    vr=v-vc(2);
    Fxc=Xudot*vcd(1); %Correct since Xudot < 0
    Fyc=Yvdot*vcd(2);
    %Fpc=.45*L*Fxc+.45*40*Fyc;
end
% Current (old)
% if curr
%     thc=22.5*pi/180; %NNE
%     PC=5e-6; %Normalized pressure of current
%     thr=psi-thc;
%     Ax=18.45*40*.4; %Frontal area of ship
%     Ay=18.45*L*.85; %Side area of ship
%     Fxc=-Ax*PC*cos(thr); %Normalized current force in x
%     Fyc=-Ay*PC*sin(thr); %Normalized current force in y
%     Fpc=.45*L*Fxc+.45*40*Fyc; %Normalized current moment in psi
% else
%     Fxc=0;
%     Fyc=0;
%     Fpc=0;
% end

% Forces and moments
if u<=0, error('u must be larger than zero'); end
beta = atan2(vr,ur);
gT   = (1/L*Tuu*ur^2 + Tun*ur*n + L*Tnn*abs(n)*n);
c    = sqrt(cun*ur*n + cnn*n^2);

%Takes current into account
gX   = 1/L*(Xuu*ur^2 + L*v*r + L*(d11-1)*vr*r + Xvv*vr^2 + Xccdd*abs(c)*c*delta^2 ...
     + Xccbd*abs(c)*c*beta*delta + L*gT*(1-t)) + Fxc;

gY   = 1/L*(Yuv*ur*vr + Yvv*abs(vr)*vr + Yccd*abs(c)*c*delta - L*u*r + L*(d22+1)*ur*r ...
     + Yccbbd*abs(c)*c*abs(beta)*beta*abs(delta) + YT*gT*L) + Fyc;     

gLN  = Nuv*ur*vr + L*Nvr*abs(vr)*r + Nccd*abs(c)*c*delta +L*d33*ur*r ...
     + Nccbbd*abs(c)*c*abs(beta)*beta*abs(delta) + L*NT*gT;
%xG implicitly assumed zero

% Original
% gX   = 1/L*(Xuu*u^2 + L*d11*v*r + Xvv*v^2 + Xccdd*abs(c)*c*delta^2 ...
%      + Xccbd*abs(c)*c*beta*delta + L*gT*(1-t))+Fxc;
% 
% gY   = 1/L*(Yuv*u*v + Yvv*abs(v)*v + Yccd*abs(c)*c*delta + L*d22*u*r ...
%      + Yccbbd*abs(c)*c*abs(beta)*beta*abs(delta) + YT*gT*L)+Fyc;     
% 
% gLN  = Nuv*u*v + L*Nvr*abs(v)*r + Nccd*abs(c)*c*delta +L*d33*u*r ...
%      + Nccbbd*abs(c)*c*abs(beta)*beta*abs(delta) + L*NT*gT+Fpc;

% Dimensional state derivative
xdot = [  gX/m11 
          gY/m22 
          gLN/(L^2*m33)
          cos(psi)*u-sin(psi)*v
          sin(psi)*u+cos(psi)*v   
          r  ];