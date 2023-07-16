function [sys, x0, str, ts] = quadrotorModel(t,x,u,flag)
    switch flag
    case 0
        [sys, x0, str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    case {2,4,9}
        sys = [];
    otherwise
        error(['Unhandled flag = ', num2str(flag)]);
    end
end
function [sys,x0,str,ts]=mdlInitializeSizes
    sizes = simsizes;
    sizes.NumContStates = 12;
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 12;
    sizes.NumInputs = 4 ;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 0;
    sys = simsizes(sizes);
    x0 = [0;0;0.2;0;0;3*pi/180;zeros(6,1)];
    str = [];
    ts = [];
end
function sys = mdlDerivatives(t,x,u)
global quad;
px = x(1);py=x(2);pz=x(3);phi = x(4);theta =x(5) ;psi =x(6);
dpx = x(7);dpy = x(8);dpz = x(9);dphi = x(10);dtheta = x(11);dpsi = x(12);

p  = [px;py;pz]; dp = [dpx;dpy;dpz];
eta = [phi;theta;psi];deta = [dphi;dtheta;dpsi];

cphi  = cos(phi); sphi = sin(phi);
cpsi = cos(psi); spsi = sin(psi);
ttheta = tan(theta); ctheta = cos(theta); stheta = sin(theta);

iphiMatrix = [1, 0, -sin(theta);
             0, cos(phi), sin(phi)*cos(theta);
             0, -sin(phi), cos(phi)*cos(theta)];

dphiMatrix = [0, cphi*ttheta*dphi+sphi*dtheta/ctheta^2, cphi*dtheta/ctheta^2-sphi*ttheta*dphi;
              0, -sphi*dphi,    -cphi*dphi;
              0, (cphi*ctheta*dphi + sphi*stheta*dtheta)/ctheta^2, (cphi*stheta*dtheta-sphi*ctheta*dphi)/ctheta^2];

I = quad.inertialMatrix;
M = iphiMatrix'*I*iphiMatrix;
iM = inv(M);
C = iphiMatrix'*(createSkewMatrix(I*iphiMatrix*deta) - I*iphiMatrix*dphiMatrix)*iphiMatrix;
Gz = [0;0;quad.g];
r3 = [cphi*stheta*cpsi+sphi*spsi;cphi*stheta*spsi-sphi*cpsi;cphi*ctheta];

F = u(1);
tau = u(2:4);

dpNext = dp;
ddpNext = 1/quad.m*(-quad.m*Gz - quad.Dp*dp + r3*F);

detaNext = deta;
ddetaNext = iM*(-C*deta - quad.Dn*deta + iphiMatrix'*tau);

sys = [dpNext;detaNext;ddpNext;ddetaNext];

end

function sys = mdlOutputs(t,x,u) 
sys = x;
end

