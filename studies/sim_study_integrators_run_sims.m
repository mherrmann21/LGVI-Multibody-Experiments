%% Simulation study comparing integrator properties
% Run all cases in one batch.
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

%% Workflow settings

% Whether to use timeit (accurate and slow) or tic/toc (fast)
runOpts.accurateTiming = false;

% Save data and plots
runOpts.saveResults = true;

% Directory where a results subfolder is created for each case
runOpts.resultsDir = fullfile(getRootFolder, "results", "runs");

% Specify cases to run
% Specify a case as a function handle in the cell array.
% The stored function handle already includes all additional options,
% e.g., if the case is conservative or dissipative.
simCases = {
    @() integrator_case_planar_pendulum(false) % Conservative
    @() integrator_case_planar_pendulum(true)  % Dissipative
    @() integrator_case_cantilever_beam(false) % Conservative
    @() integrator_case_cantilever_beam(true)  % Dissipative
    @() integrator_case_rigid_flexible % Always dissipative
    };

% Case IDs:
% 0 = pendulum
% 1 = cantilever beam
% 2 = rigid-flexible robot

%% Run requested cases

for iCase = 1:numel(simCases)
    close all
    fprintf("\nRunning integrator study case %d/%d...\n\n", iCase, numel(simCases));
    sim_study_integrators_run_case(simCases{iCase}, runOpts);
end

disp("Integrator study batch finished.")

