function caseDef = ocp_case_rigid_robot
    %% Define OCP simulation study case: rigid lab robot (ID 0)

    %% Define case

    links = systemDefLabRobotRigid();
    OCP = elara.ocp.Problem(links);
    MBSim = OCP.getSimulationObject;

    OCP.tEnd = 2;
    % Upper equilibrium
    OCP.q0 = [0; 0; 0];

    OCP.addTCPFinalTimeConstraint = false;
    OCP.qF = [];

    OCP.tPreAct  = 4*2^-3;
    OCP.tPostAct = 2*2^-5;

    % OCP.x_TCP_F = [0.7; 0; 0.3];
    OCP.qMin = ones(OCP.systemNum.nDoF, 1)*-2*pi;
    OCP.qMax = ones(OCP.systemNum.nDoF, 1)*2*pi;
    OCP.runningCostWeights = [
        1e-2/2  % Norm u
        0  % Norm u_dot
        0  % Norm u_ddot
        0  % Norm q_ddot
        5e3 % TCP error
        ];
    OCP.runningCostActive = logical(OCP.runningCostWeights);

    % No final time cost term
    OCP.finalCostActive = false(3,1);

    OCP.useSplineInputs = true;
    OCP.inputSplineOrder = 3;
    OCP.nInputSplinePoints = 25;

    OCP.qDot0 = zeros(OCP.systemNum.nDoF,1); % Initial velocity
    OCP.qDotF = zeros(OCP.systemNum.nDoF,1); % Final velocity

    % Desired TCP pose
    OCP.x_TCP_F = [0.6; 0.3; 0.3];
    % No constraints on controls
    OCP.u0 = [];
    OCP.uMin = [];
    OCP.uMax = [];

    % Compute IG from inverse dynamics
    computeInitialGuess = true;

    refDiscretization = elara.ocp.DiscretizationRK("RK4");

    %% Assign to output struct
    caseDef.systemModel = 0;
    caseDef.links = links;
    caseDef.MBSim = MBSim;
    caseDef.OCP = OCP;
    caseDef.computeInitialGuess = computeInitialGuess;
    caseDef.refDiscretization = refDiscretization;

end
