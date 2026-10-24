function r = ftppf(t, rho0, rhoT, kappa, Ts)
%FTPPF  Finite-time prescribed performance function, paper Eq. (8). Works on arrays.
%   r = (rho0 - rhoT (1 + t/Ts)) exp(kappa t/(t - Ts)) + rhoT  for 0 <= t < Ts,  r = rhoT for t >= Ts
r = rhoT * ones(size(t));
m = t < Ts;
tm = t(m);
r(m) = (rho0 - rhoT * (1 + tm / Ts)) .* exp(kappa * tm ./ (tm - Ts)) + rhoT;
end
