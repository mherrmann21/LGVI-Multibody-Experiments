%% Simulation study comparing OCP discretizations
% Run the selected cases in one batch.
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

%% Workflow settings

% Whether to restrict the comparison to the VI discretization
runOpts.debug = false;

% Whether to save data and plots or animate the initial guess
runOpts.saveResults = true;
runOpts.animateInitialGuess = true;

% Vector of time steps to investigate
% Important: Must be even divisors of the end time!
runOpts.hVec = (2.^(-6:-1:-9)).';

% Directory where a results subfolder is created for each case
runOpts.resultsDir = fullfile(getRootFolder, "results", "runs");

% Specify each case as a function handle in the cell array.
caseDefinitionFcns = {
    @ocp_case_planar_manipulator
    @ocp_case_rigid_robot
    @ocp_case_continuum_manipulator
    };

%% Run requested cases

for iCase = 1:numel(caseDefinitionFcns)
    close all
    fprintf("\nRunning OCP discretization study case %d/%d...\n\n", ...
        iCase, numel(caseDefinitionFcns));
    caseDef = caseDefinitionFcns{iCase}();
    sim_study_OCP_disc_run_case(caseDef, runOpts);
end

disp("OCP discretization study batch finished.")
