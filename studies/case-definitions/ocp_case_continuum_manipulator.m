function caseDef = ocp_case_continuum_manipulator
    %% Define OCP simulation study case: continuum manipulator (ID 1)

    %% Define case
    links = systemDefContManip_simStudy("usedTendons", [1,2,3]);
    OCP = elara.ocp.Problem(links);
    MBSim = OCP.getSimulationObject;

    OCP.tEnd = 2;
    OCP.q0 = zeros(OCP.systemNum.nDoF,1);

    OCP.qDot0 = zeros(OCP.systemNum.nDoF,1); % Initial velocity
    OCP.qDotF = zeros(OCP.systemNum.nDoF,1); % Final velocity

    OCP.runningCostWeights = [
        5e-3 % Norm u
        0    % Norm u_dot
        0    % Norm u_ddot
        0    % Norm q_ddot
        5e6  % TCP error (Running tracking error)
        ];
    OCP.runningCostActive = logical(OCP.runningCostWeights);

    % No final time cost term
    OCP.finalCostActive = false(3,1);

    OCP.addTCPFinalTimeConstraint = false;

    OCP.useSplineInputs = true;
    OCP.inputSplineOrder = 3;
    OCP.nInputSplinePoints = 40;

    % Desired TCP pose
    OCP.x_TCP_F = [0.2; 0.3; 0.4];
    % Controls constraints
    OCP.u0 = [];
    OCP.uMin = ones(OCP.systemNum.nInputs,1)*-1e-3;
    OCP.uMax = [];

    % Pre and post actuation times for the trajectory
    OCP.tPreAct  = 2*2^-5;
    OCP.tPostAct = 2*2^-5;

    % Compute IG from inverse dynamics
    computeInitialGuess = true;

    refDiscretization = elara.ocp.DiscretizationVI;

    OCP.nlpOptions.ipopt.max_iter = 125;

    % Additional options to terminate solver when the problem does not
    % seem to converge
    OCP.nlpOptions.ipopt.max_resto_iter = 20;
    OCP.nlpOptions.ipopt.diverging_iterates_tol = 1e11;

    %% Assign to output struct
    caseDef.systemModel = 1;
    caseDef.links = links;
    caseDef.MBSim = MBSim;
    caseDef.OCP = OCP;
    caseDef.computeInitialGuess = computeInitialGuess;
    caseDef.refDiscretization = refDiscretization;

end
