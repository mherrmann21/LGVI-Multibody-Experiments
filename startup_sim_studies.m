%% Startup File: Add file paths and check important dependencies

% Absolute file path of this function
funPath = mfilename("fullpath");

% Get the directory of the function = repository root path
rootPath = fileparts(funPath);

% Add important paths
addpath(genpath(fullfile(rootPath, "src")));
addpath(genpath(fullfile(rootPath, "studies")));
addpath(genpath(fullfile(rootPath, "third-party")));

% Check if ELARA toolbox is available on the path
assert(exist("MBSimulation", "file"), ...
    "ELARA toolbox functions are not found on the MATLAB path. " + ...
    "Make sure the toolbox is installed correctly.")

% Check if CasADi is installed correctly
if ~exist("casadi.MX", "class")
    warning("CasADi is not found on the MATLAB path. " + ...
    "Make sure it is installed correctly.");
end