function [sys, x0, str, ts] = attitudeInnerControllerB(t,x,u,flag)
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
    sizes.NumContStates = 3;
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 3;
    sizes.NumInputs = 12 ;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 0;
    sys = simsizes(sizes);
    x0 = [ zeros(3,1)];
    str = [];
    ts = [];
end
function sys = mdlDerivatives(t,x,u)
    vn = u(1:3);dvn = u(4:6);
    eta = u(7:9); deta = u(10:12);
    
    vt = vn - deta;
    sys = vt;
end

function sys = mdlOutputs(t,x,u) 
global quad controllerB;

vn = u(1:3);dvn = u(4:6);
eta = u(7:9);deta = u(10:12);

vt = vn - deta;
ivt = reshape(x,[],1);
sn = vt + controllerB.kn*ivt;

phi = eta(1);theta=eta(2);psi=eta(3);
dphi = deta(1);dtheta=deta(2);dpsi=deta(3);
cphi  = cos(phi); sphi = sin(phi);
cpsi = cos(psi); spsi = sin(psi);
ttheta = tan(theta); ctheta = cos(theta); stheta = sin(theta);

phiMatrix = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
             0 cos(phi) -sin(phi);
             0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

iphiMatrix = [1, 0, -sin(theta);
             0, cos(phi), sin(phi)*cos(theta);
             0, -sin(phi), cos(phi)*cos(theta)];

dphiMatrix = [0, cphi*ttheta*dphi+sphi*dtheta/ctheta^2, cphi*dtheta/ctheta^2-sphi*ttheta*dphi;
              0, -sphi*dphi,    -cphi*dphi;
              0, (cphi*ctheta*dphi + sphi*stheta*dtheta)/ctheta^2, (cphi*stheta*dtheta-sphi*ctheta*dphi)/ctheta^2];

I = quad.inertialMatrix;
M = iphiMatrix'*I*iphiMatrix;
C = iphiMatrix'*(createSkewMatrix(I*iphiMatrix*deta) - I*iphiMatrix*dphiMatrix)*iphiMatrix;


tau = M*(dvn + controllerB.kn*vt) + C*deta + controllerB.pn*sign(sn);

sys = tau;

end
