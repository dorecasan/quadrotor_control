function [sys, x0, str, ts] = attitudeOuterController(t,x,u,flag)
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
etad = u(1:3);
detad = u(4:6);
ddetad = u(7:9);

eta = u(10:12);
deta = u(13:15);

eeta = etad - eta;
deeta = detad - deta;

Un = detad + controller.Kn*eeta;
dUn = ddetad + controller.Kn*deeta;

sys = [Un;dUn];
end