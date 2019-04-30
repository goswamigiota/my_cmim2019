function dydt = eom(t, q0, param)

q = q0(1:12);
C = param.C_fun(t, q);
Cq = param.Cq_fun(t, q);
g = param.g_fun(t, q, q0(13:24));

% g_baum = g - (2*20*Cq*q0(13:24)) - (400*C);

F = force_vector(param.grav, param.bodies);

sysF = [F;
    g]; 
sysM = [param.M Cq.';
    Cq zeros(length(g))];

qdotdot = sysM\sysF;

dydt(1:12) = q0(13:24);
dydt(13:24) = qdotdot(1:12);
dydt = dydt(:);

