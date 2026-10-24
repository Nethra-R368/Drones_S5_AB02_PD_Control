%% STEP 8 : WHY THE BARRIER GAIN MUST BE CAPPED ON A SAMPLED, DELAYED LOOP
% Block 8 (implementation on the X3, docs Section 6, finding F15).
%
% The paper's law is continuous-time: the barrier gain on s grows without bound as
% |eps| -> 1. On the X3 the loop runs at 60 Hz (Gazebo pose rate) with one sample
% of transport delay. For one axis, J theta'' = u with u = -(kp/2) theta - kd_tot omega
% (q_i ~ theta/2), zero-order hold T = 1/60 s and one-sample delay:
%
%   x_{k+1} = Ad x_k + Bd u_{k-1},   u_k = -(kp/2) theta_k - kd_tot omega_k
%
% The closed loop is stable only if every eigenvalue has |lambda| < 1. kd_tot is
% the PD gain kd PLUS the barrier gain g (which multiplies s = omega + c q_v).

here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib')); figDir = fullfile(here, 'figures');

step_header(8, 'sampled-data limit of the barrier gain', { ...
    'x_{k+1} = Ad x_k + Bd u_{k-1},  u_k = -(kp/2) theta_k - kd_tot omega_k,  T = 1/60 s', ...
    'stable  <=>  max |eig| < 1'});

T = 1/60; kp = 4; Js = [0.03565 0.07035 0.09892]; names = {'roll', 'pitch', 'yaw'};   % X3 inertia about the CoM (x3_sim/x3_model.py)
kd = 0.05:0.01:8; rho = zeros(3, numel(kd)); kmax = zeros(1,3);
for a = 1:3
    J = Js(a);
    Ad = [1 T; 0 1]; Bd = [T^2/(2*J); T/J];                 % exact ZOH of a double integrator
    for k = 1:numel(kd)
        Acl = [Ad Bd; -kp/2 -kd(k) 0];
        rho(a, k) = max(abs(eig(Acl)));
    end
    kmax(a) = kd(find(rho(a,:) < 1, 1, 'last'));
end

%% Part A - hand check of the discretisation (roll axis)
fprintf('\nPart A : exact zero-order-hold discretisation, roll axis J = 0.03565 kg m^2\n');
io_print('OUT', 'Ad', [1 T; 0 1], '-', '[1 T; 0 1], T = 0.016667 s');
io_print('OUT', 'Bd', [T^2/(2*Js(1)); T/Js(1)], '-', '[T^2/2J; T/J] = [0.003896; 0.4675]');

%% Part B - largest stable total derivative gain
fprintf('\nPart B : largest stable kd_tot (kp = 4, 60 Hz, one-sample delay, ideal rate)\n');
for a = 1:3
    io_print('OUT', ['kd max ' names{a}], kmax(a), 'N m s', 'beyond this the loop oscillates and diverges');
end
fprintf(['          used on the X3: kd = 0.4 and barrier gain capped at 1.0*J_i/J_z\n' ...
         '          (roll 0.36, pitch 0.71, yaw 1.0) -> kd_tot stays well inside the stable range.\n' ...
         '          The uncapped barrier (g -> inf as |eps| -> 1) would cross it.\n']);

fig = figure('Name', 'Step 8 - sampled-data limit', 'Color', 'w', 'Position', [100 100 800 400]);
plot(kd, rho', 'LineWidth', 1.6); hold on; plot(kd, ones(size(kd)), 'k--');
grid on; xlabel('total derivative gain kd + g  [N m s/rad]'); ylabel('spectral radius max|\lambda|');
legend('roll', 'pitch', 'yaw', 'stability limit', 'Location', 'northwest'); ylim([0.8 1.3]); xlim([0 8]);
title('60 Hz, one-sample delay: the gain on s cannot grow without bound');
print(fig, fullfile(figDir, 'step8_sampled_data_limit.png'), '-dpng', '-r150');
