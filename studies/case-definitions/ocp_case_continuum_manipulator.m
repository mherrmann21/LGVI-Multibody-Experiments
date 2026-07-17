function caseDef = ocp_case_continuum_manipulator
    %% Define OCP sim study case: Continuum Manipulator (ID 1)

    SYSTEM_MDL = 1;
    OCP = OCPDefinition;

    %% Define Case
    links = systemDefContManip_simStudy("usedTendons", [1,2,3]);
    MBSim = MBSimulation(links, "displayInfo", false);

    OCP.tF = 2;
    OCP.q0 = zeros(MBSim.MBSys.nDoF,1);

    OCP.qDot0 = zeros(MBSim.MBSys.nDoF,1); % Initial velocity
    OCP.qDotF = zeros(MBSim.MBSys.nDoF,1); % Final velocity

    OCP.wRC = [
        5e-3 % Norm u
        0    % Norm u_dot
        0    % Norm u_ddot
        0    % Norm q_ddot
        1e7  % TCP error (Running tracking error)
        ];
    OCP.iRC = logical(OCP.wRC);

    % No final time cost term
    OCP.iFC = zeros(3,1);

    OCP.addTCPFinalTimeConstraint = false;

    OCP.useSplineInputs = true;
    OCP.inputSplineOrder = 3;
    OCP.nInputSplinePoints = 40;

    % Desired TCP pose
    OCP.x_TCP_F = [0.1; 0.1; links(1).L];
    OCP.x_TCP_F = [0.2; 0.3; 0.4];
    OCP.R_TCP_F = []; % Rotation arbitrary

    % Controls constraints
    OCP.u0 = [];
    OCP.uMin = ones(MBSim.MBSys.nInputs,1)*-1e-3;
    OCP.uMax = [];

    % Pre and post actuation times for the trajectory
    OCP.tPreAct  = 2*2^-5;
    OCP.tPostAct = 2*2^-5;

    % Compute IG from inverse dynamics
    computeInitialGuess = true;

    refDiscretization = OCPIntegratorVI;

    OCP.nlpOpts.ipopt.max_iter = 125;

    % Additional options to terminate solver when the problem does not
    % seem to converge
    OCP.nlpOpts.ipopt.max_resto_iter = 20;
    OCP.nlpOpts.ipopt.diverging_iterates_tol = 1e11;

    %% Assign to output struct
    caseDef.systemModel = SYSTEM_MDL;
    caseDef.links = links;
    caseDef.MBSim = MBSim;
    caseDef.OCP = OCP;
    caseDef.computeInitialGuess = computeInitialGuess;
    caseDef.refDiscretization = refDiscretization;

end
