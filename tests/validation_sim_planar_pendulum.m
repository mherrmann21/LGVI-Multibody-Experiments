%% Validate the Planar 4-Link pendulum case of the simulation study
% Integrate both with LGVI and ode45
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

%% Define System

links = systemDefPlanarNLinkPendulum();
MBSim = elara.Simulation(links, "displayInfo", true);

% Visualize reference configuration
MBSim.visualizeSystemRefConf;


%% Specify Simulation Parameters

% End time
MBSim.parameters.tEnd = 5;

% Initial configuration
q0 = zeros(MBSim.system.nDoF,1);
MBSim.parameters.q0 = q0;
MBSim.parameters.qDot0 = zeros(MBSim.system.nDoF,1);

% Visualize initial config
MBSim.visualizeSystemConfig(q0, "figureName", "visInitConf");
title("Initial Configuration")

% System inputs
MBSim.parameters.uConst = zeros(MBSim.system.nInputs,1);

%% Integration with variational integrator

MBSimVI = MBSim;

% Solver settings
MBSimVI.integrator = elara.integration.VIBroyden;
MBSimVI.integrator.h = 2^-8;
MBSimVI.integrator.JacobianIterationThreshold = 5;
MBSimVI.integrator.tolerance = 1e-11;
MBSimVI.integrator.useFirstOrderDissipation = false;

% Start integration
MBSimVI = MBSimVI.simulateSystem;

% Plotting
MBSimVI.plotAll;
MBSimVI = MBSimVI.computeEnergies;
elara.plot.energies(MBSimVI.results);

% Animate results
MBSimVI.animateSimResults("figureName", "AnimVI");


%% Integration with ODE solver

MBSimODE = MBSim;

% Solver settings
MBSimODE.integrator = elara.integration.ODEDirect;
MBSimODE.integrator.odeObject.Solver = "ode45";
MBSimODE.integrator.odeObject.AbsoluteTolerance = 1e-8;
MBSimODE.integrator.odeObject.RelativeTolerance = 1e-8;

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
