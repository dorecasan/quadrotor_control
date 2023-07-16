function [sys, x0, str, ts] = trajectoryReference(t,x,u,flag)
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
    sizes.NumOutputs = 10;
    sizes.NumInputs = 0 ;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 0;
    sys = simsizes(sizes);
    x0 = [];
    str = [];
    ts = [];
end

function sys = mdlOutputs(t,x,u) 
sys = LemniscatePath(t);
end

function ref = LemniscatePath(t)
pxd = 0.5*sin(2*pi/4*t);
dpxd = (pi*cos((pi*t)/2))/4;
ddpxd = -(pi^2*sin((pi*t)/2))/8;
pyd = cos(2*pi/8*t);
dpyd = -(pi*sin((pi*t)/4))/4;
ddpyd = -(pi^2*cos((pi*t)/4))/16;

if (t <=5)
    pzd = 1 - 0.7*exp(-0.1*t^3);
    dpzd = (21*t^2*exp(-t^3/10))/100;
    ddpzd = (21*t*exp(-t^3/10))/50 - (63*t^4*exp(-t^3/10))/1000;
else
    pzd = 1;dpzd=0;ddpzd=0;
end
psid = 0;
ref = [pxd;pyd;pzd;dpxd;dpyd;dpzd;ddpxd;ddpyd;ddpzd;psid];
end

