%% Simulation Study for Discretizations of OCPs
% Run all cases in one batch.
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

%% Workflow settings

% Debug mode restricts the comparison to the VI discretization
runOpts.debug = true;

runOpts.saveResults = true;
runOpts.animateInitialGuess = true;

% Vector of time steps to investigate
% Important: Must be even divisors of the end time!
runOpts.hVec = (2.^(-6:-1:-9)).';

% Directory where a results subfolder is created for each case
runOpts.resultsDir = fullfile(getRootFolder, "results", "runs");

% System models to run
simCases = {
    @ocp_case_planar_manipulator
    @ocp_case_rigid_robot
    @ocp_case_continuum_manipulator
    };

% System IDs:
% 0 = rigid lab robot
% 1 = continuum manipulator
% 3 = planar manipulator

%% Run requested cases

for iCase = 1:numel(simCases)
    close all
    fprintf("\nRunning OCP discretization study case %d/%d...\n\n", ...
        iCase, numel(simCases));
    sim_study_OCP_disc_run_case(simCases{iCase}, runOpts);
end

disp("OCP discretization study batch finished.")
