function [sys, x0, str, ts] = positionOuterController(t,x,u,flag)
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
    sizes.NumContStates = 0;
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 6;
    sizes.NumInputs = 15 ;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 0;
    sys = simsizes(sizes);
    x0 = [];
    str = [];
    ts = [];
end

function sys = mdlOutputs(t,x,u) 
global controller;
pd = u(1:3);
dpd = u(4:6);
ddpd = u(7:9);

p = u(10:12);
dp = u(13:15);

ep = pd - p;
dep = dpd - dp;

Up = dpd + controller.Kp*ep;
dUp = ddpd + controller.Kp*dep;

sys = [Up;dUp];
end