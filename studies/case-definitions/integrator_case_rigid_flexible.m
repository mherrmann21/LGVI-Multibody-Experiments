function caseDef = integrator_case_rigid_flexible
    %% Define integrator simulation study case: rigid-flexible system

    % intDef defines the integrators to compare. Its fields are:
    %   Name: Display name of the integrator
    %   Solver: Integrator object to use (subclass of elara.abstract.Integrator)
    %   ParamVec: Vector of accuracy parameters to run
    %     * for VI: time step h
    %     * for ODE: Scale for AbsTol and RelTol
    intDef = struct();

    % Default solver settings for VI comparison cases
    integratorVarInt = elara.integration.VIBroyden;
    integratorVarInt.JacobianIterationThreshold = 5;
    integratorVarInt.useFirstOrderDissipation = false;

    %% Rigid-flexible system
    tEnd = 3;
    hRef = 2^-16;

    % This case is always with dissipation
    dissipationCase = true;

    % Solver error margin of the VI ref. simulation
    errorMarginRef = 2*5e-12;

    links = systemDefSRFRobot();
    MBSim = elara.Simulation(links, "displayInfo", true);

    % Add Pseudo-PD Control via joint Stiffness and Dissipation
    nControlledJoints = 4;
    qDes = deg2rad([45,-45, 90,-80]);
    MBSim.system.dSys(1:nControlledJoints) = ones(nControlledJoints,1) * 20;
    MBSim.system.cSys(1:nControlledJoints) = ones(nControlledJoints,1) * 60;
    MBSim.system.qRef(1:nControlledJoints) = qDes;
    % MBSim.system.dSys(1:3) = ones(3,1) * 15;
    % MBSim.system.cSys(1:3) = ones(3,1) * 50;
    % MBSim.system.qRef(1:3) = deg2rad([45,15,-60]);

    % Define integrators to compare
    odeTols     = 10.^(-2:-1:-8);
    odeTolsLong = 10.^(-2:-1:-12);

    intDef(1).Name     = "VI-T";
    intDef(1).ParamVec = 2.^(-12:-0.5:-15);
    intDef(1).Solver = integratorVarInt;
    intDef(1).Solver.useFirstOrderDissipation = false;
    intDef(1).Solver.tolerance = 5e-12;
    intDef(1).Solver.JacobianIterationThreshold = 5;

    intDef(end+1).Name   = "VI-R";
    intDef(end).ParamVec = 2.^(-9:-1:-15);
    intDef(end).Solver = integratorVarInt;
    intDef(end).Solver.useFirstOrderDissipation = true;
    intDef(end).Solver.tolerance = 1e-11;

    % ode15s is the most efficient ode solver;
    % run it with additional tight tolerances
    % compared to the other ode solvers
    intDef(end+1).Name   = "ode15s";
    intDef(end).ParamVec = odeTolsLong;
    intDef(end).Solver   = elara.integration.ODEDirect;
    intDef(end).Solver.odeObject.Solver = "ode15s";

    % ode23s is always too slow
    % intDef(end+1).Name    = "ode23s";
    % intDef(end).ParamVec  = odeTols;
    % intDef(end).Solver    = elara.integration.ODEDirect;
    % intDef(end).Solver.odeObject.Solver = "ode23s";

    intDef(end+1).Name   = "ode23t";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = elara.integration.ODEDirect;
    intDef(end).Solver.odeObject.Solver = "ode23t";

    intDef(end+1).Name   = "RADAU";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = elara.integration.ODEDirectFunctionBased;
    intDef(end).Solver.solverFunction = @radau;

    % Sundials CVODE (stiff)
    intDef(end+1).Name   = "CVODE-S";
    intDef(end).ParamVec = odeTolsLong;
    intDef(end).Solver   = elara.integration.ODEDirect;
    intDef(end).Solver.odeObject.Solver  = "cvodesstiff";

    %% Assign to output struct
    caseDef.systemModel = 2;
    caseDef.dissipationCase = dissipationCase;
    caseDef.links = links;
    caseDef.MBSim = MBSim;
    caseDef.intDef = intDef;
    caseDef.tEnd = tEnd;
    caseDef.hRef = hRef;
    caseDef.errorMarginRef = errorMarginRef;
end
