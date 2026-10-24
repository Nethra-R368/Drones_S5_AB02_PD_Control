%% STEP 5 : THE WHOLE IDEA ON ONE AXIS (constrained vs conventional PD)
% Blocks 1-4 closed around the simplest possible plant, before the 3-D case.
%
%   Plant:   J x'' = sat(u) + d(t),  J = 1,  |u| <= 3,  d(t) = 0.3 sin(2t)
%   PD          : u = -kp x - kd v
%   Constrained : u = -kp x - kd v - [alpha/(1-e1)^2 + beta/(1-e2)^2] s - theta_hat sat(s/vs)
%   Same gains, limit, disturbance and initial state for both.

here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib')); figDir = fullfile(here, 'figures');

step_header(5, 'single-axis closed loop', { ...
    'J x'''' = sat(u) + 0.3 sin(2t),  kp = 4, kd = 2, alpha = beta = 0.5, c = 1, u_max = 3', ...
    'rho_x: 0.8 -> 0.04,  rho_v: 1.0 -> 0.10,  kappa = 1.5,  Ts = 6 s,  x(0) = 0.5, v(0) = 0.3'});

S.J = 1; S.kp = 4; S.kd = 2; S.alpha = 0.5; S.beta = 0.5; S.c = 1; S.eta1 = 0.5; S.vs = 0.05; S.umax = 3;
S.rx = [0.8 0.04 1.5 6]; S.rv = [1.0 0.10 1.5 6];
[t, Xp, Vp, Up, ~, RX, RV] = sim_axis(S, 'pd', 0.5, 0.3);
[~, Xc, Vc, Uc, TH, ~, ~]  = sim_axis(S, 'constrained', 0.5, 0.3);

fprintf('\nResults (15 s, dt = 1 ms)\n');
io_print('OUT', 'x out (PD)', 100*mean(abs(Xp) >= RX), '%', 'time x outside its envelope, conventional PD');
io_print('OUT', 'x out (C)',  100*mean(abs(Xc) >= RX), '%', 'time x outside its envelope, constrained');
io_print('OUT', 'v out (PD)', 100*mean(abs(Vp) >= RV), '%', 'time v outside its envelope, conventional PD');
io_print('OUT', 'v out (C)',  100*mean(abs(Vc) >= RV), '%', 'time v outside its envelope, constrained');
io_print('OUT', 'max|e| PD', [max(abs(Xp./RX)) max(abs(Vp./RV))], '-', '[max|e1| max|e2|] > 1: violated');
io_print('OUT', 'max|e| C',  [max(abs(Xc./RX)) max(abs(Vc./RV))], '-', '[max|e1| max|e2|] < 1: satisfied');
io_print('OUT', 'peak |u|',  [max(abs(Up)) max(abs(Uc))], '-', '[PD constrained], limit 3');
io_print('OUT', 'theta(15s)', TH(end), '-', 'adaptive estimate at the end');

fig = figure('Name', 'Step 5 - single axis', 'Color', 'w', 'Position', [100 100 1150 700]);
subplot(2,2,1); envfill(t, RX); plot(t, Xp, 'r', 'LineWidth', 1.4); plot(t, Xc, 'b', 'LineWidth', 1.6);
ylabel('x'); title('x inside \pm\rho_x(t)?'); legend('envelope', 'PD', 'constrained');
subplot(2,2,2); envfill(t, RV); plot(t, Vp, 'r', 'LineWidth', 1.4); plot(t, Vc, 'b', 'LineWidth', 1.6);
ylabel('v'); title('v inside \pm\rho_v(t)?');
subplot(2,2,3); plot(t, Up, 'r', t, Uc, 'b', 'LineWidth', 1.2); grid on;
ylabel('applied u'); xlabel('t [s]'); title('control effort (limit \pm3)'); legend('PD', 'constrained');
subplot(2,2,4); plot(t, abs(Xp./RX), 'r', t, abs(Xc./RX), 'b', 'LineWidth', 1.3); hold on;
plot(t, ones(size(t)), 'k', 'LineWidth', 1.2); ylim([0 3]); grid on;
xlabel('t [s]'); ylabel('|\epsilon_1|'); title('|\epsilon_1| < 1  \Leftrightarrow  inside'); legend('PD', 'constrained', 'boundary');
print(fig, fullfile(figDir, 'step5_single_axis.png'), '-dpng', '-r150');

function envfill(t, R)
    fill([t fliplr(t)], [R -fliplr(R)], [0.85 0.85 0.85], 'EdgeColor', 'none'); hold on; grid on;
end

function [t, X, V, U, TH, RX, RV] = sim_axis(S, mode, x, v)
    dt = 1e-3; n = 15000; t = (0:n-1)*dt;
    X = zeros(1,n); V = X; U = X; TH = X; RX = X; RV = X; th = 0;
    for k = 1:n
        tk = t(k); rx = ftppf(tk, S.rx(1), S.rx(2), S.rx(3), S.rx(4)); rv = ftppf(tk, S.rv(1), S.rv(2), S.rv(3), S.rv(4));
        u = -S.kp*x - S.kd*v;
        if strcmp(mode, 'constrained')
            e1 = min(x/rx, 0.999); e2 = min(v/rv, 0.999); s = v + S.c*x;
            u = u - (S.alpha/(1-e1)^2 + S.beta/(1-e2)^2)*s - th*max(min(s/S.vs, 1), -1);
            th = max(th + dt*(abs(s) - 2*exp(-0.5*tk)*th)/S.eta1, 0);
        end
        u = max(min(u, S.umax), -S.umax);
        X(k) = x; V(k) = v; U(k) = u; TH(k) = th; RX(k) = rx; RV(k) = rv;
        v = v + dt*(u + 0.3*sin(2*tk))/S.J;            % semi-implicit Euler
        x = x + dt*v;
    end
end
