function caseDef = ocp_case_rigid_robot
    %% Define OCP sim study case: Rigid Lab Robot (ID 0)

    SYSTEM_MDL = 0;
    OCP = OCPDefinition;

    %% Define case

    links = systemDefLabRobotRigid();
    MBSim = MBSimulation(links, "displayInfo", false);

    OCP.tF = 2;
    % Upper equilibrium
    OCP.q0 = [0, 0, 0];

    OCP.addTCPFinalTimeConstraint = false;
    OCP.qF = [];

    OCP.tPreAct  = 4*2^-3;
    OCP.tPostAct = 2*2^-5;

    % OCP.x_TCP_F = [0.7; 0; 0.3];
    OCP.qMin = ones(MBSim.MBSys.nDoF, 1)*-2*pi;
    OCP.qMax = ones(MBSim.MBSys.nDoF, 1)*2*pi;
    OCP.wRC = [
        1e-2/2  % Norm u
        0  % Norm u_dot
        0  % Norm u_ddot
        0  % Norm q_ddot
        5e3 % TCP error
        ];
    OCP.iRC = logical(OCP.wRC);

    % No final time cost term
    OCP.iFC = zeros(3,1);

    OCP.useSplineInputs = true;
    OCP.inputSplineOrder = 3;
    OCP.nInputSplinePoints = 25;

    OCP.qDot0 = zeros(MBSim.MBSys.nDoF,1); % Initial velocity
    OCP.qDotF = zeros(MBSim.MBSys.nDoF,1); % Final velocity

    % Desired TCP pose
    OCP.x_TCP_F = [0.6; 0.3; 0.3];
    OCP.R_TCP_F = []; % Rotation arbitrary

    % No constraints on controls
    OCP.u0 = [];
    OCP.uMin = [];
    OCP.uMax = [];

    % Compute IG from inverse dynamics
    computeInitialGuess = true;

    refDiscretization = OCPIntegratorRK("RK4");

    %% Assign to output struct
    caseDef.systemModel = SYSTEM_MDL;
    caseDef.links = links;
    caseDef.MBSim = MBSim;
    caseDef.OCP = OCP;
    caseDef.computeInitialGuess = computeInitialGuess;
    caseDef.refDiscretization = refDiscretization;

end
