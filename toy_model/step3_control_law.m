%% STEP 3 : CONSTRAINED PD-TYPE CONTROL LAW AND SATURATION, paper Eq. (12), (13), (22), (3)-(4)
% Block 3 of the equation flow.
%
%   INPUT  : q_v, w, theta_hat, rho_q(t), rho_w(t), gains
%   OUTPUT : sliding variable s, auxiliary torque u_cons, command u, applied torque sat(u)
%
%   s      = w + c q_v
%   u_cons = g .* s + theta_hat sat(s/varsigma)
%   u      = -kp q_v - kd w - u_cons
%   sat(u) = sign(u) min(|u|, u_max)

here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib')); figDir = fullfile(here, 'figures');
P = paper_params();

step_header(3, 'constrained PD-type control law', { ...
    's = w + c q_v       u_cons = g.*s + theta_hat sat(s/varsigma)', ...
    'u = -kp q_v - kd w - u_cons            sat(u_i) = sign(u_i) min(|u_i|, u_max)'});

%% Part A - torque at t = 0 for the paper's Part-1 state (theta_hat(0) = 0)
fprintf('\nPart A : torque at t = 0, paper Part-1 state, theta_hat = 0\n');
qv = [0.2; -0.15; -0.25]; w = [0.03; 0.02; -0.01];
[u, s, ~, ~, g] = control_law(qv, w, 0, P.rq(1), P.rw(1), P, 'paper');
u_pd = -P.kp * qv - P.kd * w;
io_print('OUT', 's',     s,    'rad/s', 'w + 0.5 q_v = [0.13 -0.055 -0.135]');
io_print('OUT', 'u_PD',  u_pd, 'N m',   '-40 q_v - 20 w = [-8.6 5.6 10.2]');
io_print('OUT', 'u_cons', g .* s, 'N m', 'g.*s = [1.937 -0.110 -0.214]');
io_print('OUT', 'u',     u,    'N m',   'u_PD - u_cons = [-10.54 5.71 10.41]');
io_print('OUT', 'sat(u)', max(min(u, P.umax), -P.umax), 'N m', 'all three axes saturate at 2.5 N m');

%% Part B - same state, three controllers
fprintf('\nPart B : command for the three controllers (same state)\n');
forms = {'paper', 'symmetric', 'pd'};
for k = 1:3
    uk = control_law(qv, w, 0, P.rq(1), P.rw(1), P, forms{k});
    io_print('OUT', ['u ' forms{k}], uk, 'N m', 'commanded torque before saturation');
end
