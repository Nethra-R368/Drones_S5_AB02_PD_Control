%% STEP 7 : FROM TORQUE TO ROTOR SPEEDS ON THE GAZEBO X3 (motor mixer)
% Block 7 of the equation flow (docs: Section 6).
%
%   INPUT  : thrust T and torque tau = [tau_x tau_y tau_z]' from the controller
%   OUTPUT : rotor speeds w_i (rad/s) for /X3/gazebo/command/motor_speed
%
%   [T; tau] = A F,   F_i = kT w_i^2,   A from the X3 SDF:
%   rotor_0 ( 0.13,-0.22) ccw,  rotor_1 (-0.13, 0.20) ccw,  rotor_2 ( 0.13, 0.22) cw,  rotor_3 (-0.13,-0.20) cw
%   tau_x = y_i F_i,  tau_y = -x_i F_i,  tau_z = -dir_i cQ F_i   (MulticopterMotorModel.cc)

here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
addpath(fullfile(here, 'lib')); figDir = fullfile(here, 'figures');

step_header(7, 'X3 motor mixer (allocation)', { ...
    '[T; tau_x; tau_y; tau_z] = A F,   F_i = kT w_i^2,   F = A^-1 [T; tau],   w_i = sqrt(F_i/kT)'});

kT = 8.54858e-6; cQ = 0.016; l = 0.225; m = 1.52; g0 = 9.81;
r = [0.13 -0.22; -0.13 0.20; 0.13 0.22; -0.13 -0.20]; sdir = [1 1 -1 -1];
A = zeros(4);
for i = 1:4, A(:, i) = [1; r(i,2); -r(i,1); -sdir(i)*cQ]; end

%% Part A - hover by hand
fprintf('\nPart A : hover, T = m g0 = 1.52*9.81 = 14.911 N, tau = 0\n');
F = A \ [m*g0; 0; 0; 0];
io_print('IN',  'A',      A, '-', 'allocation matrix built from the SDF');
io_print('OUT', 'F',      F, 'N', 'each rotor carries T/4 = 3.728 N');
io_print('OUT', 'w hover', sqrt(F/kT), 'rad/s', 'sqrt(3.728/8.54858e-6) = 660.4 rad/s (limit 800)');

%% Part B - what the first Gazebo version (v1) actually did
fprintf('\nPart B : v1 mixer pushed through the real X3 geometry\n');
c_roll = [-1 -1 1 1]; c_pitch = [1 -1 -1 1]; c_yaw = [-1 1 -1 1];
dF = kT * [c_roll'/(4*kT*l), c_pitch'/(4*kT*l), c_yaw'/(4*cQ)];
Meff = A(2:4, :) * dF;
io_print('OUT', 'M v1', Meff, '-', 'torque produced (rows x,y,z) per torque commanded (cols x,y,z)');
fprintf('          pitch command -> roll torque %.3f, pitch torque %.3f  => no pitch authority\n', Meff(1,2), Meff(2,2));
fprintf('          yaw command   -> yaw torque %.2e  (yaw term divided by cQ instead of kT*cQ)\n', Meff(3,3));
Ainv = inv(A); Mfix = A(2:4,:) * Ainv(:, 2:4);
io_print('OUT', 'M v2', Mfix, '-', 'corrected mixer F = A^-1 [T; tau]: identity');

%% Part C - torque authority at hover
Fh = m*g0/4; Fmax = kT*800^2; auth = zeros(1,3);
for ax = 1:3
    e = zeros(4,1); e(ax+1) = 1; dfa = Ainv*e;
    auth(ax) = min((Fmax - Fh)/max(dfa), Fh/max(-dfa));
end
fprintf('\nPart C : largest torque the X3 can produce while hovering\n');
io_print('OUT', 'tau max', auth, 'N m', '[roll pitch yaw]');
fprintf('          the paper''s yaw disturbance (0.15 N m) exceeds the X3 yaw authority (%.3f N m)\n', auth(3));

fig = figure('Name', 'Step 7 - mixer', 'Color', 'w', 'Position', [100 100 1100 420]);
cm = [min(1, 1 + linspace(-1,1,64)'), 1 - abs(linspace(-1,1,64)'), min(1, 1 - linspace(-1,1,64)')];
lab = {'cmd \tau_x','cmd \tau_y','cmd \tau_z'}; labr = {'got \tau_x','got \tau_y','got \tau_z'};
Ms = {Meff, Mfix}; tt = {'v1 mixer on the real X3 (ideal = identity)', 'v2 mixer F = A^{-1}[T;\tau]'};
for p = 1:2
    subplot(1, 2, p); imagesc(Ms{p}, [-1 1]); colormap(gca, cm); colorbar;
    set(gca, 'XTick', 1:3, 'XTickLabel', lab, 'YTick', 1:3, 'YTickLabel', labr);
    for i = 1:3, for j = 1:3
        text(j, i, sprintf('%+.2f', Ms{p}(i,j)), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end, end
    title(tt{p});
end
print(fig, fullfile(figDir, 'step7_x3_mixer.png'), '-dpng', '-r150');
