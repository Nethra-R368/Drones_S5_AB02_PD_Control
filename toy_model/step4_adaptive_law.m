%% STEP 4 : ADAPTIVE ESTIMATE OF THE LUMPED DISTURBANCE BOUND, paper Eq. (14)
% Block 4 of the equation flow.
%
%   INPUT  : ||s(t)||, eta1, eta2(t)
%   OUTPUT : theta_hat(t), used in u_cons as theta_hat sat(s/varsigma)
%
%   theta_hat_dot = (||s|| - eta2(t) theta_hat) / eta1 ,   eta2(t) = 2 exp(-0.5 t),
%   integral_0^inf eta2 dt = eta_bar = 4 < inf   (condition of Theorem 1)

here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib')); figDir = fullfile(here, 'figures');
P = paper_params();

step_header(4, 'adaptive law', { ...
    'theta_hat_dot = (||s|| - eta2(t) theta_hat)/eta1,   eta2 = 2 exp(-0.5 t),   eta1 = 0.1'});

%% Part A - first Euler step by hand
fprintf('\nPart A : one Euler step (dt = 1 ms) from theta_hat = 0 with ||s|| = 0.195 (Part-1 state)\n');
s0 = norm([0.13 -0.055 -0.135]); dt = 1e-3;
th1 = 0 + dt * (s0 - 2 * 0) / P.eta1;
io_print('IN',  '||s||',     s0,  'rad/s', 'sqrt(0.13^2 + 0.055^2 + 0.135^2) = 0.1954');
io_print('OUT', 'theta(1ms)', th1, 'N m',  '0.001*0.1954/0.1 = 0.001954');

%% Part B - three typical ||s|| signals
fprintf('\nPart B : theta_hat driven by three typical ||s(t)|| signals\n');
t = 0:dt:60; n = numel(t);
s1 = 0.3 * exp(-0.8 * t);                              % converges to zero
s2 = s1 + 0.004 * abs(sin(1.5 * t));                   % small persistent residual
th = zeros(3, n);
for k = 1:n-1
    e2 = 2 * exp(-0.5 * t(k));
    th(1,k+1) = max(th(1,k) + dt*(s1(k) - e2*th(1,k))/P.eta1, 0);
    th(2,k+1) = max(th(2,k) + dt*(s2(k) - e2*th(2,k))/P.eta1, 0);
    th(3,k+1) = max(th(3,k) + dt*(s2(k) - max(e2, 0.2)*th(3,k))/P.eta1, 0);
end
io_print('CHK', 'int eta2', trapz(t, 2*exp(-0.5*t)), '-', 'integral of eta2 over 60 s (analytic limit 4)');
io_print('OUT', 'theta(20s)', th(:, round(20/dt)+1), 'N m', '[s->0, residual, residual + floor 0.2]');
io_print('OUT', 'theta(60s)', th(:, end),            'N m', 'case 2 keeps growing; the floor (case 3) bounds it');

fig = figure('Name', 'Step 4 - adaptive law', 'Color', 'w', 'Position', [100 100 1100 400]);
subplot(1,2,1); plot(t, s1, t, s2, 'LineWidth', 1.4); grid on; xlim([0 20]);
xlabel('t [s]'); ylabel('||s||'); legend('decays to 0', 'small residual'); title('input to the adaptive law');
subplot(1,2,2); plot(t, th(1,:), t, th(2,:), t, th(3,:), 'LineWidth', 1.6); grid on;
xlabel('t [s]'); ylabel('\theta_{hat}');
legend('paper \eta_2, ||s||\rightarrow0', 'paper \eta_2, residual', '\eta_2 \geq 0.2, residual', 'Location', 'northwest');
title('adaptive estimate \theta_{hat}(t)');
print(fig, fullfile(figDir, 'step4_adaptive_law.png'), '-dpng', '-r150');
