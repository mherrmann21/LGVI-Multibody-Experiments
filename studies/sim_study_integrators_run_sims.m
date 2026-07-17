%% Simulation study comparing integrator properties
% Run the selected cases in one batch.
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

% Whether to save data and plots
runOpts.saveResults = true;

% Directory where a results subfolder is created for each case
runOpts.resultsDir = fullfile(getRootFolder, "results", "runs");

% Specify each case as a function handle in the cell array.
% Each function handle includes all additional options,
% e.g., if the case is conservative or dissipative.
caseDefinitionFcns = {
    @() integrator_case_planar_pendulum(false) % Conservative
    @() integrator_case_planar_pendulum(true)  % Dissipative
    @() integrator_case_cantilever_beam(false) % Conservative
    @() integrator_case_cantilever_beam(true)  % Dissipative
    @() integrator_case_rigid_flexible % Always dissipative
    };

%% Run requested cases

for iCase = 1:numel(caseDefinitionFcns)
    close all
    fprintf("\nRunning integrator study case %d/%d...\n\n", ...
        iCase, numel(caseDefinitionFcns));
    caseDef = caseDefinitionFcns{iCase}();
    sim_study_integrators_run_case(caseDef, runOpts);
end

disp("Integrator study batch finished.")
