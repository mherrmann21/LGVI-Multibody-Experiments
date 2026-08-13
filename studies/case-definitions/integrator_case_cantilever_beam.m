function caseDef = integrator_case_cantilever_beam(dissipationCase)
    %% Define integrator simulation study case: cantilever beam
    arguments
        % Whether the dissipative or conservative case is used
        dissipationCase (1,1) logical
    end

    % intDef defines the integrators to compare. Its fields are:
    %   Name: Display name of the integrator
    %   Solver: Integrator object to use (subclass of elara.abstract.Integrator)
    %   ParamVec: Vector of accuracy parameters to run
    %     * for VI: time step h
    %     * for ODE: Scale for AbsTol and RelTol
    intDef = struct();

    %% Default solver settings for VI comparison cases
    integratorVarInt = elara.integration.VIBroyden;
    integratorVarInt.JacobianIterationThreshold = 5;
    integratorVarInt.useFirstOrderDissipation = false;

    %% HK24 cantilever beam
    tEnd = 0.2;
    hRef = 2^-19;

    % Solver error margin of the VI ref. simulation
    errorMarginRef = 2*1e-14;

    if dissipationCase
        links = systemDefCantileverBeamHK24;
    else
        links = systemDefCantileverBeamHK24("d", 0);
    end
    MBSim = elara.Simulation(links, "displayInfo", true);

    % Align beam with global x axis
    R0 = [
        +0 0 1
        +0 1 0
        -1 0 0
        ];
    MBSim.system.g0 = elara.SE3.matrix(R0, zeros(3,1));

    % No gravity
    MBSim.parameters.g = 0;

    % External forces (at beam tip)
    fMax = [0.5 0 0 0 2 2 ]' * 0.5;  % Max. force
    fTEnd = 0.05;                    % Force impulse end time
    fNodes = [zeros(6,links.nSegments-1),fMax];
    MBSim.parameters.externalWrench_s = MBSim.parameters.externalWrench_s.addWrench( ...
        0, fTEnd, 4, fNodes);

    % Define integrators to compare
    intDef(1).Name     = "VI-T";
    intDef(1).ParamVec = 2.^(-15:-0.5:-18.5);
    intDef(1).Solver = integratorVarInt;
    intDef(1).Solver.useFirstOrderDissipation = false;

    if dissipationCase
        intDef(1).Solver.tolerance = 5e-14;
    else
        intDef(1).Solver.tolerance = 1e-14;
    end

    if dissipationCase
        odeTols     = 10.^(-2:-1:-7);
        odeTolsLong = 10.^(-2:-1:-11);

        intDef(end+1).Name   = "VI-R";
        intDef(end).ParamVec = 2.^(-14:-1:-18.5);
        intDef(end).Solver = integratorVarInt;
        intDef(end).Solver.useFirstOrderDissipation = true;
        intDef(end).Solver.tolerance = 1e-12;

        % ode15s way too slow for the non-dissipative case/even gives
        % warning that tolerances can't be met
        intDef(end+1).Name   = "ode15s";
        intDef(end).ParamVec = odeTolsLong;
        intDef(end).Solver   = elara.integration.ODEDirect;
        intDef(end).Solver.odeObject.Solver = "ode15s";

        % Sundials CVODE (stiff)
        % Same as ode15s
        intDef(end+1).Name   = "CVODE-S";
        intDef(end).ParamVec = odeTolsLong;
        intDef(end).Solver   = elara.integration.ODEDirect;
        intDef(end).Solver.odeObject.Solver  = "cvodesstiff";
    else
        odeTols     = 10.^(-2:-1:-8);
    end

    % ode23s always too slow

    intDef(end+1).Name   = "ode23t";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = elara.integration.ODEDirect;
    intDef(end).Solver.odeObject.Solver = "ode23t";

    intDef(end+1).Name   = "RADAU";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = elara.integration.ODEDirectFunctionBased;
    intDef(end).Solver.solverFunction = @radau;


    %% Assign to output struct
    caseDef.systemModel = 1;
    caseDef.dissipationCase = dissipationCase;
    caseDef.links = links;
    caseDef.MBSim = MBSim;
    caseDef.intDef = intDef;
    caseDef.tEnd = tEnd;
    caseDef.hRef = hRef;
    caseDef.errorMarginRef = errorMarginRef;
end
