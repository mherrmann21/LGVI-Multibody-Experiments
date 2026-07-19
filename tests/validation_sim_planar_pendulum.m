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
MBSim = MBSimulation(links, "displayInfo", true);

% Visualize reference configuration
MBSim.visualizeSystemRefConf;


%% Specify Simulation Parameters

% End time
MBSim.simPars.tEnd = 5;

% Initial configuration
q0 = zeros(MBSim.MBSys.nDoF,1);
MBSim.simPars.q0 = q0;
MBSim.simPars.qDot0 = zeros(MBSim.MBSys.nDoF,1);

% Visualize initial config
MBSim.visualizeSystemConfig(q0, "figureName", "visInitConf");
title("Initial Configuration")

% System inputs
MBSim.simPars.uConst = zeros(MBSim.MBSys.nInputs,1);

%% Integration with variational integrator

MBSimVI = MBSim;

% Solver settings
MBSimVI.solver = MBSimIntegratorVarIntBroyden;
MBSimVI.solver.h = 2^-8;
MBSimVI.solver.JacobianIterationThreshold = 5;
MBSimVI.solver.errorMargin = 1e-11;
MBSimVI.solver.aTrapez = 1/2;

% Start integration
MBSimVI = MBSimVI.simulateSystem;

% Plotting
MBSimVI.plotAll;
MBSimVI = MBSimVI.computeEnergies;
plotEnergies(MBSimVI.simRes);

% Animate results
MBSimVI.animateSimResults("figureName", "AnimVI");


%% Integration with ODE solver

MBSimODE = MBSim;

% Solver settings
MBSimODE.solver = MBSimIntegratorODEDirect;
MBSimODE.solver.odeObject.Solver = "ode45";
MBSimODE.solver.odeObject.AbsoluteTolerance = 1e-8;
MBSimODE.solver.odeObject.RelativeTolerance = 1e-8;

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
