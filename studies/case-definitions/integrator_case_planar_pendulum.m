function caseDef = integrator_case_planar_pendulum(dissipationCase)
    %% Define integrator simulation study case: planar pendulum
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

    %% Planar pendulum
    tEnd = 5;
    hRef = 2^-14;

    % Solver error margin of the VI ref. simulation
    errorMarginRef = 2*1e-10;

    if dissipationCase
        links = systemDefPlanarNLinkPendulum("d", 2.5e-2);
    else
        links = systemDefPlanarNLinkPendulum("d", 0);
    end

    MBSim = MBSimulation(links, "displayInfo", true);

    % Define integrators to compare
    odeTols = 10.^(-3:-0.5:-8);

    intDef(1).Name     = "VI-T";
    intDef(1).ParamVec = 2.^(-6:-0.5:-13.5);
    intDef(1).Solver = integratorVarInt;
    intDef(1).Solver.aTrapez = 1/2;
    intDef(1).Solver.errorMargin = 1e-10;

    if any(MBSim.MBSys.dSys)
        intDef(end+1).Name   = "VI-R";
        intDef(end).ParamVec = 2.^(-6:-0.5:-13.5);
        intDef(end).Solver = integratorVarInt;
        intDef(end).Solver.aTrapez = 0;
        intDef(end).Solver.errorMargin = 1e-10;
    end

    % Almost always worse
    intDef(end+1).Name   = "ode23";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = MBSimIntegratorODEDirect;
    intDef(end).Solver.odeObject.Solver = "ode23";

    intDef(end+1).Name   = "ode113";
    intDef(end).ParamVec = 10.^(-3:-1:-8);
    intDef(end).Solver   = MBSimIntegratorODEDirect;
    intDef(end).Solver.odeObject.Solver = "ode113";

    intDef(end+1).Name   = "ode45";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = MBSimIntegratorODEDirect;
    intDef(end).Solver.odeObject.Solver = "ode45";

    intDef(end+1).Name   = "ode78";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = MBSimIntegratorODEDirect;
    intDef(end).Solver.odeObject.Solver = "ode78";

    % ode89 performs similar to ode78, but with more "uneven"
    % dependence on tolerance
    intDef(end+1).Name   = "ode89";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = MBSimIntegratorODEDirect;
    intDef(end).Solver.odeObject.Solver = "ode89";

    % Radau always significantly worse
    intDef(end+1).Name   = "RADAU";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = MBSimIntegratorODEDirectFunctionBased;
    intDef(end).Solver.solverFunction = @radau;

    % Sundials CVODE (nonstiff)
    intDef(end+1).Name   = "CVODE-N";
    intDef(end).ParamVec = odeTols;
    intDef(end).Solver   = MBSimIntegratorODEDirect;
    intDef(end).Solver.odeObject.Solver = "cvodesnonstiff";


    %% Assign to output struct
    caseDef.systemModel = 0;
    caseDef.dissipationCase = dissipationCase;
    caseDef.links = links;
    caseDef.MBSim = MBSim;
    caseDef.intDef = intDef;
    caseDef.tEnd = tEnd;
    caseDef.hRef = hRef;
    caseDef.errorMarginRef = errorMarginRef;
end
