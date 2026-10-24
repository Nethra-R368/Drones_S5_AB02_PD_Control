%% RUN_ALL - runs every step of the AB02 toy model and saves the printed output
%  Golestani et al., "A Simple Structure Constrained Attitude Control for Rigid
%  Bodies: A PD-Type Control", IEEE Access 10 (2022) 10202-10209.
%
%  Usage (from any folder):  >> run('<repo>/toy_model/run_all.m')
%  Output: figures/*.png and results_log.txt (complete printed output of this run)
%  No toolbox is needed (plain MATLAB, R2016b or newer).

close all; clc;
ab02_here = fileparts(mfilename('fullpath')); if isempty(ab02_here), ab02_here = pwd; end
ab02_log = fullfile(ab02_here, 'results_log.txt');
if exist(ab02_log, 'file'), delete(ab02_log); end
diary(ab02_log);
fprintf('AB02 toy model | %s | MATLAB %s\n', char(datetime('now')), version);
ab02_steps = {'step1_ftppf', 'step2_normalised_error', 'step3_control_law', 'step4_adaptive_law', ...
              'step5_single_axis', 'step6_spacecraft_closed_loop', 'step7_x3_mixer', 'step8_sampled_data_limit'};
for ab02_i = 1:numel(ab02_steps)
    ab02_t0 = tic;
    run(fullfile(ab02_here, [ab02_steps{ab02_i} '.m']));
    fprintf('(%s finished in %.1f s)\n', ab02_steps{ab02_i}, toc(ab02_t0));
    close all;
end
fprintf('\nAll steps finished. Figures: %s\n', fullfile(ab02_here, 'figures'));
diary off;
