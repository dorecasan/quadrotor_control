function [sys, x0, str, ts] = positionInnerControllerA(t,x,u,flag)
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
    sizes.NumInputs = 22 ;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 0;
    sys = simsizes(sizes);
    x0 = [zeros(1,3)];
    str = [];
    ts = [];
end

function sys = mdlDerivatives(t,x,u)
Up = u(1:3);dUp = u(4:6);
p = u(7:9);eta = u(10:12);dp = u(13:15);deta = u(16:18);
ddpd = u(19:21); psid = u(22);
phi = eta(1);theta=eta(2);psi=eta(3);
eUp = Up - dp;

sys = eUp;
end


function sys = mdlOutputs(t,x,u) 
global quad controllerA;

Up = u(1:3);dUp = u(4:6);
p = u(7:9);eta = u(10:12);dp = u(13:15);deta = u(16:18);
ddpd = u(19:21); psid = u(22);
phi = eta(1);theta=eta(2);psi=eta(3);

eUp = Up - dp;
eps = x;
Gz = [0;0;quad.g];

rF = controllerA.K1p*eUp + controllerA.K2p*eps + quad.mR*(dUp + Gz) + quad.Dp*dp;

fx  = rF(1);fy=rF(2);fz=rF(3);

F = fz/(cos(phi)*cos(theta));
thetad = atan(1/fz*(fx*cos(psid)+fy*sin(psid)));
phid = atan(cos(thetad)/fz*(fx*sin(psid)-fy*cos(psid)));

sys = [F;phid;thetad];

end
