%% STEP 2 : NORMALISED ERRORS AND THE BARRIER GAIN, paper Eq. (11) and (13)
% Block 2 of the equation flow.
%
%   INPUT  : quaternion vector part q_v, body rate w, envelopes rho_q(t), rho_w(t)
%   OUTPUT : eps1 = q_v / rho_q,  eps2 = w / rho_w   (constraint  <=>  |eps| < 1)
%            barrier gain  g = alpha/(1-eps1)^2 + beta/(1-eps2)^2   (per axis)
%
% In the project: paper_reproduction.py -> control(),  x3_sim/controllers.py -> FixedController.update()

here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib')); figDir = fullfile(here, 'figures');
if ~exist(figDir, 'dir'), mkdir(figDir); end
P = paper_params();

step_header(2, 'normalised errors and barrier gain', { ...
    'eps1 = q_v / rho_q(t)        eps2 = w / rho_w(t)        |eps| < 1  <=>  inside envelope', ...
    'g_i  = alpha/(1 - eps1_i)^2 + beta/(1 - eps2_i)^2     (paper form, one-sided)', ...
    'g_i  = alpha/(1 - eps1_i^2)^2 + beta/(1 - eps2_i^2)^2  (symmetric variant)'});

%% Part A - the paper's Part-1 initial state at t = 0
fprintf('\nPart A : paper Part-1 initial state at t = 0 (rho_q = 0.3, rho_w = 0.4)\n');
qv = [0.2; -0.15; -0.25]; w = [0.03; 0.02; -0.01];
e1 = qv / P.rq(1); e2 = w / P.rw(1);
g_p = P.alpha ./ (1 - e1).^2 + P.beta ./ (1 - e2).^2;
g_s = P.alpha ./ (1 - e1.^2).^2 + P.beta ./ (1 - e2.^2).^2;
io_print('IN',  'q_v',  qv, '-', 'quaternion vector part');
io_print('IN',  'w',    w,  'rad/s', 'body rate');
io_print('OUT', 'eps1', e1, '-', '[0.2 -0.15 -0.25]/0.3 = [0.6667 -0.5 -0.8333]: all |eps1| < 1, inside');
io_print('OUT', 'eps2', e2, '-', '[0.03 0.02 -0.01]/0.4 = [0.075 0.05 -0.025]');
io_print('OUT', 'g paper', g_p, '-', 'axis 1: 1.5/(1/3)^2 + 1.2/0.925^2 = 13.5 + 1.403 = 14.903');
io_print('OUT', 'g symm.', g_s, '-', 'symmetric form');
fprintf(['          note: axis 3 is CLOSEST to its boundary (|eps1| = 0.833) but the paper form gives it\n' ...
         '          the SMALLEST gain (1.59) because eps1 is negative - the one-sided property.\n']);

%% Part B - gain over the whole envelope
e = linspace(-0.95, 0.95, 801);
fig = figure('Name', 'Step 2 - barrier gain', 'Color', 'w', 'Position', [100 100 1100 420]);
subplot(1, 2, 1);
semilogy(e, P.alpha ./ (1 - e).^2, 'b', 'LineWidth', 2); hold on;
semilogy(e, P.alpha ./ (1 - e.^2).^2, 'g--', 'LineWidth', 2);
plot(e1, P.alpha ./ (1 - e1).^2, 'bo', 'MarkerFaceColor', 'b');
grid on; xlabel('\epsilon_1'); ylabel('\alpha-part of the gain on s');
legend('paper \alpha(1-\epsilon)^{-2}', 'symmetric \alpha(1-\epsilon^2)^{-2}', 'Part A axes', 'Location', 'north');
title('barrier gain (\alpha = 1.5)');
subplot(1, 2, 2);
s = 0.05;
plot(e, -(P.alpha ./ (1 - e).^2) * s, 'b', 'LineWidth', 2); hold on;
plot(e, -(P.alpha ./ (1 - e.^2).^2) * s, 'g--', 'LineWidth', 2);
plot(e, -P.alpha * s * ones(size(e)), 'k:', 'LineWidth', 1.2);
grid on; ylim([-30 1]); xlabel('\epsilon_1'); ylabel('-\alpha g s  [N m]');
legend('paper form', 'symmetric form', 'constant gain', 'Location', 'southwest');
title('extra restoring torque for s = 0.05');
print(fig, fullfile(figDir, 'step2_barrier_gain.png'), '-dpng', '-r150');
