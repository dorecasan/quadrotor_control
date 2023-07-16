global quad  controller controllerA controllerB Mode;
Mode = 1;
quad.g = 9.81;
quad.m = 1.79;
quad.Ixx = 0.03;
quad.Iyy = 0.03;
quad.Izz = 0.04;
quad.Ixy = 0;quad.Ixz = 0;quad.Iyz = 0;quad.Iyz = 0;
quad.Dp = diag([0.002,0.002,0.004]);
quad.Dn = diag([0.002,0.002,0.004]);
quad.inertialMatrix = [quad.Ixx quad.Ixy quad.Ixz;
                       quad.Ixy quad.Iyy quad.Iyz;
                       quad.Ixz quad.Iyz quad.Izz];
quad.mR = 1.253;
controller.Kup = diag([4.0,4.0,9.5]);
controller.Kip = diag([0.004,0.004,1.0]);
controller.Kp = diag([1.5375,1.5375,5.3]);
controller.Kun = diag([0.475,0.475,0.9]);
controller.Kin = diag([0.001,0.001,0.005]);
controller.Kn = diag([4.7368,4.7368,7.7778]);
controller.Lp = 10;
controller.Ln = 10;
controller.Nn = diag(0.001*ones(1,controller.Ln));
controller.kn = 0.002;
controller.an = 0.001;
controller.Np = diag(0.35*ones(1,controller.Lp));
controller.kp = 0.075;
controller.ap = 0.001;
controller.Vp = rand(10,controller.Lp)*2-1;
controller.Vn = rand(10,controller.Ln)*2-1;



controllerA.K1p = diag([2.25, 2.25, 5.25]);
controllerA.K2p = diag([0.004, 0.004, 3.0]);
controllerA.K3p = diag([1.7957, 1.7957, 1.7957]);
controllerA.K1n = diag([0.475, 0.475, 0.9]);
controllerA.K2n = diag([0.001, 0.001, 0.005]);
controllerA.K3n = diag([4.7368, 4.7368, 7.7778]);



controllerB.Lp = 7;
controllerB.c = [-1.5, -1, -0.5, 0, 0.5, 1, 1.5]';
controllerB.kw = 4.5;
controllerB.sigma = [5, 5, 5, 5, 5, 5, 5]';
controllerB.pw = 1e-6;
controllerB.delta = diag([2.55,2.55,3.55]);
controllerB.kn = 15;
controllerB.A = diag(0.15*ones(1,controllerB.Lp));
controllerB.pn = 1e-4;
controllerB.kv = 5.257;


