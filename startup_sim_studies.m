%% Startup file: add repository paths and check important dependencies

% Absolute path of this startup script
startupPath = mfilename("fullpath");

% The script is located in the repository root
rootPath = fileparts(startupPath);

% Add important paths
addpath(genpath(fullfile(rootPath, "src")));
addpath(genpath(fullfile(rootPath, "studies")));
addpath(genpath(fullfile(rootPath, "tests")));
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
