function caseDef = integrator_case_cantilever_beam(dissipationCase)
    %% Define integrator simulation study case: cantilever beam
    arguments
        % Whether the dissipative or conservative case is used
        dissipationCase (1,1) logical
    end

    % intDef defines the integrators to compare. Its fields are:
    %   Name: Display name of the integrator
    %   Solver: Solver object to use (subclass of MBSimIntegrator)
    %   ParamVec: Vector of accuracy parameters to run
    %     * for VI: time step h
    %     * for ODE: Scale for AbsTol and RelTol
    intDef = struct();

    %% Default solver settings for VI comparison cases
    integratorVarInt = MBSimIntegratorVarIntBroyden;
    integratorVarInt.JacobianIterationThreshold = 5;
    integratorVarInt.aTrapez = 1/2;

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
    MBSim = MBSimulation(links, "displayInfo", true);

    % Align beam with global x axis
    R0 = [
        +0 0 1
        +0 1 0
        -1 0 0
        ];
    MBSim.MBSys.g0 = SE3Matrix(R0, zeros(3,1));

    % No gravity
    MBSim.simPars.g = 0;

    % External forces (at beam tip)
    fMax = [0.5 0 0 0 2 2 ]' * 0.5;  % Max. force
    fTEnd = 0.05;                    % Force impulse end time
    fNodes = [zeros(6,links.nSeg-1),fMax];
    MBSim.simPars.extWrench_s = MBSim.simPars.extWrench_s.addWrench( ...
        0, fTEnd, 4, fNodes);

    % Define integrators to compare
    intDef(1).Name     = "VI-T";
    intDef(1).ParamVec = 2.^(-15:-0.5:-18.5);
    intDef(1).Solver = integratorVarInt;
    intDef(1).Solver.aTrapez = 1/2;

    if dissipationCase
        intDef(1).Solver.errorMargin = 5e-14;
    else
        intDef(1).Solver.errorMargin = 1e-14;
    end

    if dissipationCase
        odeTols     = 10.^(-2:-1:-7);
        odeTolsLong = 10.^(-2:-1:-11);

        intDef(end+1).Name   = "VI-R";
        intDef(end).ParamVec = 2.^(-14:-1:-18.5);
        intDef(end).Solver = integratorVarInt;
        intDef(end).Solver.aTrapez = 0;
        intDef(end).Solver.errorMargin = 1e-12;

        % ode15s way too slow for the non-dissipative case/even gives
        % warning that tolerances can't be met
        intDef(end+1).Name   = "ode15s";
        intDef(end).ParamVec = odeTolsLong;
        intDef(end).Solver   = MBSimIntegratorODEDirect;
        intDef(end).Solver.odeObject.Solver = "ode15s";

        % Sundials CVODE (stiff)
        % Same as ode15s
        intDef(end+1).Name   = "CVODE-S";
        intDef(end).ParamVec = odeTolsLong;
        intDef(end).Solver   = MBSimIntegratorODEDirect;
        intDef(end).Solver.odeObject.Solver  = "cvodesstiff";
    else
        odeTols     = 10.^(-2:-1:-8);
    end

    % ode23s always too slow

    intDef(end+1).Name   = "ode23t";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = MBSimIntegratorODEDirect;
    intDef(end).Solver.odeObject.Solver = "ode23t";

    intDef(end+1).Name   = "RADAU";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = MBSimIntegratorODEDirectFunctionBased;
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
