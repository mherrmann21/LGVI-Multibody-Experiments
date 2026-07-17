function caseDef = ocp_case_planar_manipulator
    %% Define OCP simulation study case: planar manipulator (ID 3)

    OCP = OCPDefinition;

    %% Define case

    links = systemDefPlanarNLinkPendulum("nLinks", 2, "d", 0);
    MBSim = MBSimulation(links, "displayInfo", true);

    OCP.tF = 1;
    OCP.q0 = [pi/2, 0];

    OCP.qDot0 = zeros(MBSim.MBSys.nDoF,1); % Initial velocity
    OCP.qDotF = zeros(MBSim.MBSys.nDoF,1); % Final velocity

    OCP.qMin = -2*pi*ones(2,1);
    OCP.qMax = +2*pi*ones(2,1);

    if 1
        % To reproduce the simple example in [Obe08]
        OCP.wRC = [
            1/2  % Norm u
            0  % Norm u_dot
            0  % Norm u_ddot
            0  % Norm q_ddot
            0  % TCP error
            ];
        % To make sure all OCPs converge to the same solution
        OCP.qMin(2) = -4;
        OCP.qMax(2) = 0.1;
    else
        % For various other tests
        OCP.wRC = [
            1/2  % Norm u
            1/2  % Norm u_dot
            1/2*1e-3  % Norm u_ddot
            1/2*1e-3  % Norm q_ddot
            1e5  % TCP error
            ];
    end
    OCP.iRC = logical(OCP.wRC);

    % No final time cost term
    OCP.iFC = zeros(3,1);

    % Input limits: Not necessary, but significantly improve VI
    % convergence for some reason (with splines)
    OCP.uMin = ones(2,1)*-30;
    OCP.uMax = ones(2,1)*+30;

    OCP.useSplineInputs = true;
    OCP.inputSplineOrder = 3;
    OCP.nInputSplinePoints = 15;

    OCP.qF = [-pi/2, 0];
    OCP.addTCPFinalTimeConstraint = false;

    % Use zero initial guess
    computeInitialGuess = false;

    refDiscretization = OCPIntegratorRK("RK4");

    %% Assign to output struct
    caseDef.systemModel = 3;
    caseDef.links = links;
    caseDef.MBSim = MBSim;
    caseDef.OCP = OCP;
    caseDef.computeInitialGuess = computeInitialGuess;
    caseDef.refDiscretization = refDiscretization;
end
