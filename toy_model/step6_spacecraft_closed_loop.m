%% STEP 6 : FULL 3-AXIS CLOSED LOOP - reproduction of the paper's Section IV
% Blocks 1-4 + plant Eq. (1) + saturation Eq. (3)-(4).  Independent MATLAB
% re-implementation of paper_reproduction.py (cross-check of the Python numbers).
%
%   q_v_dot = 1/2 (q4 I + [q_v]x) w,  q4_dot = -1/2 q_v' w,  J w_dot = -w x J w + sat(u) + d
%   RK4, dt = 1 ms, torque held over each step (zero-order hold)
%   Part 1 : constrained law, Ts = 15 s and 10 s
%   Part 2 : constrained (paper) vs symmetric vs conventional PD, Ts = 15 s, rho_q0 = 0.36

here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib')); figDir = fullfile(here, 'figures');
P = paper_params();

step_header(6, 'spacecraft closed loop (paper Section IV)', { ...
    'q_v_dot = 1/2(q4 I + [q_v]x) w,  q4_dot = -1/2 q_v''w,  J w_dot = -w x Jw + sat(u) + d(t)', ...
    'd(t) = [0.05 sin t, 0.1 sin 1.2t, 0.15 sin 1.5t]'' N m,   u_max = 2.5 N m'});

fprintf('\nPart A : Part 1, q(0) = [0.2 -0.15 -0.25 0.9354], w(0) = [0.03 0.02 -0.01]\n');
for Ts = [15 10]
    P1 = P; P1.rq(4) = Ts; P1.rw(4) = Ts;
    R = sim_att(P1, 'paper', [0.2 -0.15 -0.25 0.9354], [0.03 0.02 -0.01]);
    io_print('OUT', sprintf('Ts=%d', Ts), [R.e1max R.e2max R.vq R.vw R.settle], '-', ...
        '[max|eps1| max|eps2| q-out% w-out% settling-s]');
end

fprintf('\nPart B : Part 2, q(0) = [0.3 -0.2 -0.3 0.8832], w(0) = [-0.02 -0.02 0.025], Ts = 15 s, rho_q0 = 0.36\n');
P2 = P; P2.rq(1) = 0.36;
forms = {'paper', 'symmetric', 'pd'};
RR = cell(1, 3);
for k = 1:3
    RR{k} = sim_att(P2, forms{k}, [0.3 -0.2 -0.3 0.8832], [-0.02 -0.02 0.025]);
    R = RR{k};
    io_print('OUT', forms{k}, [R.e1max R.e2max R.vq R.vw R.atlim R.settle R.energy], '-', ...
        '[max|e1| max|e2| q-out% w-out% at-limit% settle-s energy]');
end

fig = figure('Name', 'Step 6 - Part 2', 'Color', 'w', 'Position', [60 60 1250 760]);
cols = [0 0.447 0.741; 0.85 0.1 0.1; 0.2 0.6 0.2];
ttl = {'constrained (paper Eq. 13)', '', 'conventional PD'};
for k = [1 3]
    R = RR{k}; c = 1 + (k == 3);
    subplot(3, 2, c);
    fill([R.t fliplr(R.t)], [R.rq -fliplr(R.rq)], [0.85 0.85 0.85], 'EdgeColor', 'none'); hold on;
    for i = 1:3, plot(R.t, R.q(i,:), 'Color', cols(i,:), 'LineWidth', 1.3); end
    grid on; ylabel('q_v'); title([ttl{k} ': quaternion']);
    subplot(3, 2, c + 2);
    fill([R.t fliplr(R.t)], [R.rw -fliplr(R.rw)], [0.85 0.85 0.85], 'EdgeColor', 'none'); hold on;
    for i = 1:3, plot(R.t, R.w(i,:), 'Color', cols(i,:), 'LineWidth', 1.3); end
    grid on; ylabel('\omega [rad/s]'); title('angular velocity');
    subplot(3, 2, c + 4);
    for i = 1:3, plot(R.t, R.u(i,:), 'Color', cols(i,:), 'LineWidth', 1.1); hold on; end
    grid on; ylabel('sat(u) [N m]'); xlabel('t [s]'); title('applied torque');
end
print(fig, fullfile(figDir, 'step6_part2_constrained_vs_pd.png'), '-dpng', '-r150');

fig2 = figure('Name', 'Step 6 - eps2', 'Color', 'w', 'Position', [60 60 1000 380]);
for k = 1:3, plot(RR{k}.t, RR{k}.e2abs, 'LineWidth', 1.4); hold on; end
plot(RR{1}.t, ones(size(RR{1}.t)), 'k', 'LineWidth', 1.2);
grid on; xlabel('t [s]'); ylabel('max_i |\epsilon_{2i}(t)|'); ylim([0 1.6]);
legend('constrained (paper)', 'constrained (symmetric)', 'conventional PD', 'boundary');
title('angular-velocity constraint: |\epsilon_2| < 1 means inside the envelope');
print(fig2, fullfile(figDir, 'step6_part2_eps2.png'), '-dpng', '-r150');

% ------------------------------------------------------------------------
function [dq, dw] = rhs(q, w, u, d, J)
    qv = q(1:3); q4 = q(4);
    S = [0 -qv(3) qv(2); qv(3) 0 -qv(1); -qv(2) qv(1) 0];
    dq = [0.5*(q4*eye(3) + S)*w; -0.5*(qv'*w)];
    dw = J \ (-cross(w, J*w) + u + d);
end

function R = sim_att(P, form, q0, w0)
    dt = 1e-3; n = 20000; rec = 10; m = n/rec;
    q = q0(:)/norm(q0); w = w0(:); th = 0;
    dist = @(tt) [0.05*sin(tt); 0.10*sin(1.2*tt); 0.15*sin(1.5*tt)];
    R.t = zeros(1,m); R.q = zeros(3,m); R.w = zeros(3,m); R.u = zeros(3,m);
    R.rq = zeros(1,m); R.rw = zeros(1,m); R.e2abs = zeros(1,m);
    vq = 0; vw = 0; atlim = 0; e1max = 0; e2max = 0; energy = 0; settle = 0; j = 0;
    for k = 0:n-1
        t = k*dt;
        rq = ftppf(t, P.rq(1), P.rq(2), P.rq(3), P.rq(4));
        rw = ftppf(t, P.rw(1), P.rw(2), P.rw(3), P.rw(4));
        qv = q(1:3);
        [u, s, e1, e2] = control_law(qv, w, th, rq, rw, P, form);
        us = max(min(u, P.umax), -P.umax);
        vq = vq + any(abs(qv) >= rq); vw = vw + any(abs(w) >= rw);
        atlim = atlim + any(abs(u) >= P.umax);
        e1max = max(e1max, max(abs(e1))); e2max = max(e2max, max(abs(e2)));
        energy = energy + sum(us.^2)*dt;
        if norm(qv) > 0.01, settle = t; end
        if mod(k, rec) == 0
            j = j + 1; R.t(j) = t; R.q(:,j) = qv; R.w(:,j) = w; R.u(:,j) = us;
            R.rq(j) = rq; R.rw(j) = rw; R.e2abs(j) = max(abs(e2));
        end
        [k1q, k1w] = rhs(q, w, us, dist(t), P.J);
        [k2q, k2w] = rhs(q + dt/2*k1q, w + dt/2*k1w, us, dist(t+dt/2), P.J);
        [k3q, k3w] = rhs(q + dt/2*k2q, w + dt/2*k2w, us, dist(t+dt/2), P.J);
        [k4q, k4w] = rhs(q + dt*k3q, w + dt*k3w, us, dist(t+dt), P.J);
        q = q + dt/6*(k1q + 2*k2q + 2*k3q + k4q);
        w = w + dt/6*(k1w + 2*k2w + 2*k3w + k4w);
        q = q/norm(q);
        if ~strcmp(form, 'pd')
            th = max(th + dt*(norm(s) - 2*exp(-0.5*t)*th)/P.eta1, 0);
        end
    end
    R.vq = 100*vq/n; R.vw = 100*vw/n; R.atlim = 100*atlim/n; R.e1max = e1max; R.e2max = e2max;
    R.energy = energy; R.settle = settle;
end
