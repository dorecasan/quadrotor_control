function [sys, x0, str, ts] = attitudeOuterControllerB(t,x,u,flag)
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
    sizes.NumOutputs = 6;
    sizes.NumInputs = 15 ;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 0;
    sys = simsizes(sizes);
    x0 = [zeros(3,1)];
    str = [];
    ts = [];
end

function sys = mdlDerivatives(t,x,u) 
    etad = u(1:3);
    detad = u(4:6);
    ddetad = u(7:9);
    
    eta = u(10:12);
    deta = u(13:15);
    
    eeta =  etad - eta;
    deeta = detad - deta;
      
    sys = eeta;

end

function sys = mdlOutputs(t,x,u) 
    global controllerB;
    etad = u(1:3);
    detad = u(4:6);
    ddetad = u(7:9);
    
    eta = u(10:12);
    deta = u(13:15);
    
    eeta =  etad - eta;
    deeta = detad - deta;
    
    ieeta = reshape(x,[],1);
    sw = eeta + controllerB.kw*ieeta;
    vn = detad + controllerB.kw*eeta + controllerB.pw*sign(sw);
    dvn = ddetad + controllerB.kw*deeta;
    
    sys = [vn;dvn];

end