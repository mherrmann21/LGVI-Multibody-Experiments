%% Validate the Cantilever beam case of the simulation study
% * Perform simulations with some of the solvers used in the simulation study
% * Compare simulation results to known results from literature
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

%% Define System

link  = systemDefCantileverBeamHK24("d", 0);
MBSim = elara.Simulation(link, "displayInfo", true);

% Align beam with global x axis
R0 = [
    0  0 1
    0  1 0
    -1 0 0
    ];
MBSim.system.g0 = elara.SE3.matrix(R0, zeros(3,1));

% Visualize reference configuration
MBSim.visualizeSystemRefConf;


%% Specify Simulation Parameters

% End time
MBSim.parameters.tEnd = 0.2;

% Initial configuration
q0 = zeros(MBSim.system.nDoF,1);
MBSim.parameters.q0 = q0;
MBSim.parameters.qDot0 = zeros(MBSim.system.nDoF,1);

% Visualize initial config
MBSim.visualizeSystemConfig(q0, "figureName", "visInitConf");
title("Initial Configuration")

% No gravity
MBSim.parameters.g = 0;

% External forces (at beam tip)
fMax = [0.5 0 0 0 2 2 ]' * 0.5;  % Max. force
fTEnd = 0.05;                    % Force impulse end time

fNodes = [zeros(6,link.nSegments-1),fMax];
MBSim.parameters.externalWrench_s = MBSim.parameters.externalWrench_s.addWrench( ...
    0, fTEnd, 4, fNodes);


%% Integration with variational integrator

MBSimVI = MBSim;

% Working maximum time steps:
% -11.5 with dissipation, a = 0
% -15 without dissipation

% Solver settings
MBSimVI.integrator = elara.integration.VIBroyden;
MBSimVI.integrator.h = 2^-15;
MBSimVI.integrator.JacobianIterationThreshold = 5;
MBSimVI.integrator.tolerance = 1e-10;
MBSimVI.integrator.useFirstOrderDissipation = false; % Irrelevant for conservative case

% Start integration
MBSimVI = MBSimVI.simulateSystem;

% Plotting
MBSimVI.plotAll;
MBSimVI = MBSimVI.computeEnergies;
elara.plot.energies(MBSimVI.results);

% Animate results
MBSimVI.animateSimResults("figureName", "AnimVI");



%% Validate simulation with literature results
% From https://github.com/plkinon/ph_cosserat_rods/
% Note that the axes from the github results have been switched due to
% different local frame conventions

ph_repo_path = fullfile(getRootFolder, "third-party", "ph_cosserat_rods");

% HK24 results (from repo)
resHK24 = readtable(fullfile(ph_repo_path, fullfile( ...
    "reference_results","example_02_HerrmannKotyczka2024", ...
    "ex02_cantilever_reference_results.csv" ...
    )));

% KEB25 results
resKEB25 = readtable(fullfile(ph_repo_path, ...
    "results\example_02_dynamic_cantilever\ex02_cantilever_results.csv"));

% Velocities
figure("Name", "Lit. Comparison Velocities", "NumberTitle", "off");
tiledlayout("TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(resHK24.time, resHK24.tip_velocity_B_2, "LineWidth", 2);
hold on;
plot(resKEB25.time, resKEB25.tip_velocity_B_2, "-.", "LineWidth", 2);
plot(MBSimVI.results.tout, squeeze(MBSimVI.results.eta(4,end,:)), "--", "LineWidth", 2);
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("$\eta_4$ in m/s", "Interpreter", "latex");
grid on;
legend("HK24", "KEB25", "Current", "Interpreter", "latex");

nexttile;
plot(resHK24.time, -resHK24.tip_velocity_B_1, "LineWidth", 2);
hold on;
plot(resKEB25.time, -resKEB25.tip_velocity_B_1, "-.", "LineWidth", 2);
plot(MBSimVI.results.tout, squeeze(MBSimVI.results.eta(5,end,:)), "--", "LineWidth", 2);
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("$\eta_5$ in m/s", "Interpreter", "latex");
grid on;

nexttile;
plot(resHK24.time, resHK24.tip_velocity_B_3, "LineWidth", 2);
hold on;
plot(resKEB25.time, resKEB25.tip_velocity_B_3, "-.", "LineWidth", 2);
plot(MBSimVI.results.tout, squeeze(MBSimVI.results.eta(6,end,:)), "--", "LineWidth", 2);
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("$\eta_6$ in m/s", "Interpreter", "latex");
grid on;

% Tip Positions
figure("Name", "Lit. Comparison Positions", "NumberTitle", "off");
tiledlayout("TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(resHK24.time, resHK24.tip_position_I_1, "LineWidth", 2);
hold on;
plot(resKEB25.time, resKEB25.tip_position_I_1, "-.", "LineWidth", 2);
plot(MBSimVI.results.tout, squeeze(MBSimVI.results.g(1,4,end,:)), "--", "LineWidth", 2);
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("$x$ in m", "Interpreter", "latex");
grid on;
legend("HK24", "KEB25", "Current", "Interpreter", "latex");

nexttile;
plot(resHK24.time, resHK24.tip_position_I_2, "LineWidth", 2);
hold on;
plot(resKEB25.time, resKEB25.tip_position_I_2, "-.", "LineWidth", 2);
plot(MBSimVI.results.tout, squeeze(MBSimVI.results.g(2,4,end,:)), "--", "LineWidth", 2);
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("$y$ in m", "Interpreter", "latex");
grid on;

nexttile;
plot(resHK24.time, resHK24.tip_position_I_3, "LineWidth", 2);
hold on;
plot(resKEB25.time, resKEB25.tip_position_I_3, "-.", "LineWidth", 2);
plot(MBSimVI.results.tout, squeeze(MBSimVI.results.g(3,4,end,:)), "--", "LineWidth", 2);
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("$z$ in m", "Interpreter", "latex");
grid on;


%% Validate integration with ODE solvers

MBSimODE = MBSim;

% Solver settings
MBSimODE.integrator = elara.integration.ODEDirect;
MBSimODE.integrator.odeObject.Solver = "cvodesstiff";
MBSimODE.integrator.odeObject.RelativeTolerance = 1e-3;
MBSimODE.integrator.odeObject.AbsoluteTolerance = 1e-3;

% Start integration
MBSimODE = MBSimODE.simulateSystem;

% Plotting
MBSimODE.plotAll;
MBSimODE = MBSimODE.computeEnergies;
elara.plot.energies(MBSimODE.results);

% Animate results
MBSimODE.animateSimResults("figureName", "AnimODE");


%% Validate integration with radau

MBSimODE.integrator = elara.integration.ODEDirectFunctionBased;
MBSimODE.integrator.solverFunction = @radau;
MBSimODE.integrator.solverOptions = rdpset('RelTol',1e-3, 'AbsTol', 1e-3);

% Start integration
MBSimODE = MBSimODE.simulateSystem;

% Plotting
MBSimODE.plotAll;
MBSimODE = MBSimODE.computeEnergies;
elara.plot.energies(MBSimODE.results);

% Animate results
MBSimODE.animateSimResults("figureName", "AnimODE");

%% End script
disp("Finished.")
