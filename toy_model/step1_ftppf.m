%% STEP 1 : FINITE-TIME PRESCRIBED PERFORMANCE FUNCTION (FTPPF), paper Eq. (8)
% Block 1 of the equation flow (docs: Mathematical Models, Section 3).
%
%   INPUT  : time t, design parameters rho0 (initial bound), rhoT (final bound),
%            kappa (shape), Ts (settling time)
%   OUTPUT : envelope rho(t) that every |q_i| (or |w_i|) must stay below
%
%   rho(t) = (rho0 - rhoT (1 + t/Ts)) exp(kappa t / (t - Ts)) + rhoT ,  0 <= t < Ts
%   rho(t) = rhoT                                                    ,  t >= Ts
%
% In the project: paper_reproduction.py -> ftppf(),  x3_sim/controllers.py -> ftppf()

here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib')); figDir = fullfile(here, 'figures');
if ~exist(figDir, 'dir'), mkdir(figDir); end
P = paper_params();

step_header(1, 'finite-time prescribed performance function', { ...
    'rho(t) = (rho0 - rhoT(1 + t/Ts)) exp(kappa t/(t - Ts)) + rhoT   for 0 <= t < Ts', ...
    'rho(t) = rhoT                                                    for t >= Ts'});

%% Part A - toy example from the documentation (attitude envelope of the paper at t = 2 s)
fprintf('\nPart A : rho_q at t = 2 s with the paper values (rho0 = 0.3, rhoT = 0.008, kappa = 2.5, Ts = 15 s)\n');
t = 2; r = ftppf(t, P.rq(1), P.rq(2), P.rq(3), P.rq(4));
io_print('IN',  't',      t,                         's', 'time');
io_print('OUT', 'bracket', P.rq(1) - P.rq(2)*(1 + t/P.rq(4)), '-', '0.3 - 0.008*(1 + 2/15) = 0.290933');
io_print('OUT', 'exp term', exp(P.rq(3)*t/(t - P.rq(4))), '-', 'exp(2.5*2/(2-15)) = exp(-0.3846) = 0.680712');
io_print('OUT', 'rho_q(2)', r,                       '-', '0.290933*0.680712 + 0.008 = 0.206042');

%% Part B - the finite-time property and the effect of each design knob
fprintf('\nPart B : finite-time property\n');
t = linspace(0, 12, 2401); Ts = 8;
r_ft  = ftppf(t, 0.3, 0.008, 2.5, Ts);
r_exp = (0.3 - 0.008) * exp(-0.5 * t) + 0.008;            % classical PPF, Eq. (7)
[~, iTs] = min(abs(t - Ts));
io_print('CHK', 'rho(0)',     r_ft(1),   '-', 'equals rho0 = 0.3');
io_print('CHK', 'rho(Ts)',    r_ft(iTs), '-', 'equals rhoT = 0.008 exactly at Ts = 8 s');
io_print('CHK', 'exp PPF(Ts)', r_exp(iTs), '-', 'Eq. (7) is still 1.67 x rhoT at Ts');
io_print('CHK', 'monotone',   all(diff(r_ft) <= 1e-12), '-', 'rho never increases (1 = true)');

fig = figure('Name', 'Step 1 - FTPPF', 'Color', 'w', 'Position', [100 100 1100 420]);
subplot(1, 2, 1);
plot(t, r_exp, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.4); hold on;
plot(t, r_ft, 'b', 'LineWidth', 2); plot([Ts Ts], [0 0.32], 'r:', 'LineWidth', 1.2);
grid on; xlabel('t [s]'); ylabel('\rho(t)');
legend('exponential PPF, Eq. (7)', 'FTPPF, Eq. (8)', 'T_s = 8 s', 'Location', 'northeast');
title('FTPPF reaches \rho_T exactly at T_s');
subplot(1, 2, 2); hold on; cols = lines(4); Tss = [4 6 8 10];
for k = 1:4, plot(t, ftppf(t, 0.3, 0.008, 2.5, Tss(k)), 'Color', cols(k,:), 'LineWidth', 1.6); end
plot(t, ftppf(t, 0.3, 0.008, 0.5, 8), 'k--', 'LineWidth', 1.1);
plot(t, ftppf(t, 0.3, 0.008, 5, 8), 'k-.', 'LineWidth', 1.1);
grid on; xlabel('t [s]'); ylabel('\rho(t)');
legend('T_s=4', 'T_s=6', 'T_s=8', 'T_s=10', '\kappa=0.5 (T_s=8)', '\kappa=5 (T_s=8)');
title('T_s sets the arrival time, \kappa the shape');
print(fig, fullfile(figDir, 'step1_ftppf.png'), '-dpng', '-r150');
