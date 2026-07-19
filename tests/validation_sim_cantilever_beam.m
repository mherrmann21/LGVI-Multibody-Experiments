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
MBSim = MBSimulation(link, "displayInfo", true);

% Align beam with global x axis
R0 = [
    0  0 1
    0  1 0
    -1 0 0
    ];
MBSim.MBSys.g0 = SE3Matrix(R0, zeros(3,1));

% Visualize reference configuration
MBSim.visualizeSystemRefConf;


%% Specify Simulation Parameters

% End time
MBSim.simPars.tEnd = 0.2;

% Initial configuration
q0 = ones(MBSim.MBSys.nDoF,1)*0;
MBSim.simPars.q0 = q0;
MBSim.simPars.qDot0 = zeros(MBSim.MBSys.nDoF,1);

% Visualize initial config
MBSim.visualizeSystemConfig(q0, "figureName", "visInitConf");
title("Initial Configuration")

% No gravity
MBSim.simPars.g = 0;

% External forces (at beam tip)
fMax = [0.5 0 0 0 2 2 ]' * 0.5;  % Max. force
fTEnd = 0.05;                    % Force impulse end time

fNodes = [zeros(6,link.nSeg-1),fMax];
MBSim.simPars.extWrench_s = MBSim.simPars.extWrench_s.addWrench( ...
    0, fTEnd, 4, fNodes);


%% Integration with variational integrator

MBSimVI = MBSim;

% Working maximum time steps:
% -11.5 with dissipation, a = 0
% -15 without dissipation

% Solver settings
MBSimVI.solver = MBSimIntegratorVarIntBroyden;
MBSimVI.solver.h = 2^-15;
MBSimVI.solver.JacobianIterationThreshold = 5;
MBSimVI.solver.errorMargin = 1e-10;
MBSimVI.solver.aTrapez = 1/2; % Irrelevant for conservative case

% Start integration
MBSimVI = MBSimVI.simulateSystem;

% Plotting
MBSimVI.plotAll;
MBSimVI = MBSimVI.computeEnergies;
plotEnergies(MBSimVI.simRes);

% Animate results
MBSimVI.animateSimResults("figureName", "AnimVI");



%% Validate simulation with literature results
% From https://github.com/plkinon/ph_cosserat_rods/
% Note that the axes from the github results have been switched due to
% different local frame conventions

ph_repo_path = fullfile(getRootFolder, "third-party", "ph_cosserat_rods");

% HK24 results (from repo)
resHK24 = readtable(fullfile(ph_repo_path, ...
    "reference_results\example_02_HerrmannKotyczka2024\ex02_cantilever_reference_results.csv"));

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
plot(MBSimVI.simRes.tout, squeeze(MBSimVI.simRes.eta(4,end,:)), "--", "LineWidth", 2);
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("$\eta_4$ in m/s", "Interpreter", "latex");
grid on;
legend("HK24", "KEB25", "Current", "Interpreter", "latex");

nexttile;
plot(resHK24.time, -resHK24.tip_velocity_B_1, "LineWidth", 2);
hold on;
plot(resKEB25.time, -resKEB25.tip_velocity_B_1, "-.", "LineWidth", 2);
plot(MBSimVI.simRes.tout, squeeze(MBSimVI.simRes.eta(5,end,:)), "--", "LineWidth", 2);
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("$\eta_5$ in m/s", "Interpreter", "latex");
grid on;

nexttile;
plot(resHK24.time, resHK24.tip_velocity_B_3, "LineWidth", 2);
hold on;
plot(resKEB25.time, resKEB25.tip_velocity_B_3, "-.", "LineWidth", 2);
plot(MBSimVI.simRes.tout, squeeze(MBSimVI.simRes.eta(6,end,:)), "--", "LineWidth", 2);
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
plot(MBSimVI.simRes.tout, squeeze(MBSimVI.simRes.g(1,4,end,:)), "--", "LineWidth", 2);
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("$x$ in m", "Interpreter", "latex");
grid on;
legend("HK24", "KEB25", "Current", "Interpreter", "latex");

nexttile;
plot(resHK24.time, resHK24.tip_position_I_2, "LineWidth", 2);
hold on;
plot(resKEB25.time, resKEB25.tip_position_I_2, "-.", "LineWidth", 2);
plot(MBSimVI.simRes.tout, squeeze(MBSimVI.simRes.g(2,4,end,:)), "--", "LineWidth", 2);
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("$y$ in m", "Interpreter", "latex");
grid on;

nexttile;
plot(resHK24.time, resHK24.tip_position_I_3, "LineWidth", 2);
hold on;
plot(resKEB25.time, resKEB25.tip_position_I_3, "-.", "LineWidth", 2);
plot(MBSimVI.simRes.tout, squeeze(MBSimVI.simRes.g(3,4,end,:)), "--", "LineWidth", 2);
xlabel("time $t$ in s", "Interpreter", "latex");
ylabel("$z$ in m", "Interpreter", "latex");
grid on;


%% Validate integration with ODE solvers

MBSimODE = MBSim;

% Solver settings
MBSimODE.solver = MBSimIntegratorODEDirect;
MBSimODE.solver.odeObject.Solver = "ode23t";
MBSimODE.solver.odeObject.AbsoluteTolerance = 1e-3;
MBSimODE.solver.odeObject.RelativeTolerance = 1e-3;

MBSimODE.solver.odeObject.Solver = "cvodesstiff";
MBSimODE.solver.odeObject.RelativeTolerance = 1e-3;
MBSimODE.solver.odeObject.AbsoluteTolerance = 1e-3;

% Start integration
MBSimODE = MBSimODE.simulateSystem;

% Plotting
MBSimODE.plotAll;
MBSimODE = MBSimODE.computeEnergies;
plotEnergies(MBSimODE.simRes);

% Animate results
MBSimODE.animateSimResults("figureName", "AnimODE");


%% Validate integration with radau

MBSimODE.solver = MBSimIntegratorODEDirectFunctionBased;
MBSimODE.solver.solverFunction = @radau;
MBSimODE.solver.solverOptions = rdpset('RelTol',1e-3, 'AbsTol', 1e-3);

% Start integration
MBSimODE = MBSimODE.simulateSystem;

% Plotting
MBSimODE.plotAll;
MBSimODE = MBSimODE.computeEnergies;
plotEnergies(MBSimODE.simRes);

% Animate results
MBSimODE.animateSimResults("figureName", "AnimODE");

%% End script
disp("Finished.")
