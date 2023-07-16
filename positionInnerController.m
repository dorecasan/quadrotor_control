function [sys, x0, str, ts] = positionInnerController(t,x,u,flag)
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
    global controller;
    sizes = simsizes;
    sizes.NumContStates = controller.Lp*3+3;
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 3;
    sizes.NumInputs = 22 ;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 0;
    sys = simsizes(sizes);
    x0 = [0.0*ones(1, controller.Lp*3), zeros(1,3)];
    str = [];
    ts = [];
end
function sys = mdlDerivatives(t,x,u)
global controller;

Up = u(1:3);dUp = u(4:6);
p = u(7:9);eta = u(10:12);dp = u(13:15);deta = u(16:18);

gammap  = [1;dp;eta;dUp];
vp = controller.Vp'*gammap;
sigmap = tanh(vp);

eUp = Up - dp;

Wp = reshape(x(1:controller.Lp*3),[],3);
dWp = controller.Np*(sigmap*eUp' - controller.kp*norm(eUp)*Wp);

sys(1:controller.Lp*3) = reshape(dWp,1,[]);
sys(controller.Lp*3+1:controller.Lp*3+3) = eUp;
end

function sys = mdlOutputs(t,x,u) 
global quad controller;

Up = u(1:3);dUp = u(4:6);
p = u(7:9);eta = u(10:12);dp = u(13:15);deta = u(16:18);
ddpd = u(19:21); psid = u(22);
phi = eta(1);theta=eta(2);psi=eta(3);
eUp = Up - dp;

Wp = reshape(x(1:controller.Lp*3),[],3);
eps = x(controller.Lp*3+1:end);

Gz = [0;0;quad.g];
gammap  = [1;dp;eta;dUp];
vp = controller.Vp'*gammap;
sigmap = tanh(vp);

rF = controller.Kup*eUp + controller.Kip*eps + quad.mR*(ddpd + Gz) + Wp'*sigmap + controller.ap*sign(eUp);
fx  = rF(1);fy=rF(2);fz=rF(3);
F = fz/(cos(phi)*cos(theta));
thetad = atan(1/fz*(fx*cos(psid)+fy*sin(psid)));
phid = atan(cos(thetad)/fz*(fx*sin(psid)-fy*cos(psid)));

sys = [F;phid;thetad];
end
