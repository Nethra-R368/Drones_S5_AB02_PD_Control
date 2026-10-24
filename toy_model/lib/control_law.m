function [u, s, e1, e2, g] = control_law(qv, w, th, rq, rw, P, form)
%CONTROL_LAW  Constrained PD-type law, paper Eq. (11)-(13) with Eq. (22).
%   form : 'paper' (alpha/(1-e)^2, one-sided) | 'symmetric' (alpha/(1-e^2)^2) | 'pd' (u_cons = 0)
e1 = qv / rq;  e2 = w / rw;  s = w + P.c * qv;
u = -P.kp * qv - P.kd * w;
switch form
    case 'paper'
        g = P.alpha ./ (1 - min(e1, 0.999)).^2 + P.beta ./ (1 - min(e2, 0.999)).^2;
    case 'symmetric'
        e1c = max(min(e1, 0.999), -0.999); e2c = max(min(e2, 0.999), -0.999);
        g = P.alpha ./ (1 - e1c.^2).^2 + P.beta ./ (1 - e2c.^2).^2;
    otherwise
        g = zeros(3, 1);
end
if ~strcmp(form, 'pd')
    u = u - (g .* s + th * max(min(s / P.vs, 1), -1));
end
end
