function [sys, x0, str, ts] = attitudeInnerController(t,x,u,flag)
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
    sizes.NumContStates = controller.Ln*3+3;
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 3;
    sizes.NumInputs = 12 ;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 0;
    sys = simsizes(sizes);
    x0 = [0.01*ones(1, controller.Ln*3), zeros(1,3)];
    str = [];
    ts = [];
end
function sys = mdlDerivatives(t,x,u)
global controller;

Un = u(1:3);dUn = u(4:6);
eta = u(7:9);deta = u(10:12);

gamman  = [1;eta;deta;dUn];
vn = controller.Vn'*gamman;
sigman = tanh(vn);

eUn = Un - deta;

Wn = reshape(x(1:controller.Ln*3),[],3);
dWp = controller.Nn*(sigman*eUn' - controller.kn*norm(eUn)*Wn);

sys(1:controller.Ln*3) = reshape(dWp,1,[]);
sys(controller.Ln*3+1:controller.Ln*3+3) = eUn;
end

function sys = mdlOutputs(t,x,u) 
global quad controller;

Un = u(1:3);dUn = u(4:6);
eta = u(7:9);deta = u(10:12);

phi = eta(1);theta=eta(2);psi=eta(3);
phiMatrix = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
             0 cos(phi) -sin(phi);
             0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

eUn = Un - deta;

Wn = reshape(x(1:controller.Ln*3),[],3);
eps = x(controller.Ln*3+1:end);

Gz = [0;0;quad.g];
gamman  = [1;eta;deta;dUn];
vn = controller.Vn'*gamman;
sigman = tanh(vn);

tau = phiMatrix'*(controller.Kun*eUn +controller.Kin*eps + Wn'*sigman + controller.an*sign(eUn));

sys = tau;
end
