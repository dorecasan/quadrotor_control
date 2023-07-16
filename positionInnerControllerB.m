function [sys, x0, str, ts] = positionInnerControllerB(t,x,u,flag)
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
    global controllerB;
    sizes = simsizes;
    sizes.NumContStates = controllerB.Lp*3;
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 3;
    sizes.NumInputs = 22 ;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 0;
    sys = simsizes(sizes);
    x0 = 0.5*ones(1,controllerB.Lp*3);
    str = [];
    ts = [];
end

function sys = mdlDerivatives(t,x,u)
    global controllerB;
    p = u(1:3);eta = u(4:6);dp = u(7:9);deta = u(10:12);
    pd = u(13:15);dpd = u(16:18);ddpd=u(19:21);psid = u(22);
    
    ep = pd - p;
    dep = dpd - dp;
    
    gamma = dep + controllerB.delta*ep;
    Xin = [ep;dep;pd;dpd;ddpd];
    
    
    P = zeros(controllerB.Lp,1);

    for i = 1:1:controllerB.Lp
        P(i) = exp(-norm(Xin-controllerB.c(i))^2/(controllerB.sigma(i)^2));
    end
    
    dW = controllerB.A*P*gamma';
    sys = reshape(dW,1,[]);
end

function sys = mdlOutputs(t,x,u) 
    global quad controllerB;
    p = u(1:3);eta = u(4:6);dp = u(7:9);deta = u(10:12);
    pd = u(13:15);dpd = u(16:18);ddpd=u(19:21);psid = u(22);
    
    phi = eta(1);theta=eta(2);psi=eta(3);
    dphi = deta(1);dtheta=deta(2);dpsi=deta(3);
    
    ep = pd - p;
    dep = dpd - dp;
    
    gamma = dep + controllerB.delta*ep;
    
     Xin = [ep;dep;pd;dpd;ddpd];


    P = zeros(controllerB.Lp,1);
    for i = 1:1:controllerB.Lp
        P(i) = exp(-norm(Xin-controllerB.c(i))^2/(2*controllerB.sigma(i)^2));
    end
    W = reshape(x,[],3);
     f = W'*P;
 
    % f =  controllerB.delta*(dpd-dp);
    
    B = 1/quad.mR*diag([1,1,cos(phi)*cos(theta)]);
    iB = inv(B);
    Gz = [0;0;quad.g];
    
    rF = quad.mR*(Gz + f + ddpd) + controllerB.kv*gamma + quad.Dp*dp;
 

    fx  = rF(1);fy=rF(2);fz=rF(3);
    
    F = fz/(cos(phi)*cos(theta));
    thetad = atan(1/fz*(fx*cos(psid)+fy*sin(psid)));
    phid = atan(cos(thetad)/fz*(fx*sin(psid)-fy*cos(psid)));
    
    sys = [F;phid;thetad];

end


% function sys = mdlOutputs(t,x,u) 
%     global quad controllerB;
%     p = u(1:3);eta = u(4:6);dp = u(7:9);deta = u(10:12);
%     pd = u(13:15);dpd = u(16:18);ddpd=u(19:21);psid = u(22);
% 
%     phi = eta(1);theta=eta(2);psi=eta(3);
%     dphi = deta(1);dtheta=deta(2);dpsi=deta(3);
% 
%     ep = pd - p;
%     dep = dpd - dp;
% 
%     gamma = dep + controllerB.delta*ep;
%     Xin = [ep;dep;pd;dpd;ddpd];
% 
% 
%     P = zeros(controllerB.Lp,1);
%     for i = 1:1:controllerB.Lp
%         P(i) = exp(-norm(Xin-controllerB.c(i))^2/(2*controllerB.sigma(i)^2));
%     end
%     W = reshape(x,[],3);
%     % f = W'*P;
%     f =  controllerB.delta*(dpd-dp);
% 
%     B = 1/quad.mR*diag([1,1,cos(phi)*cos(theta)]);
%     iB = inv(B);
%     Gz = [0;0;quad.g];
% 
%     uA = iB*(Gz + f + controllerB.kv*gamma + ddpd);
%     F = uA(3);
%     phid = angle(asin(uA(1)*sin(psid) - uA(2)*cos(psid)));
%     thetad = angle(asin((uA(1)*cos(psid)+uA(2)*sin(psid))/cos(phid)));
%     sys = [F;phid;thetad];
% end
