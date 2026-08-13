function link = systemDefContManip_simStudy(opts)
    %% Define MBS System: One-link continuum manipulator
    arguments
        opts.d      (6,1) double = ones(6,1)*0.5e-3;
        opts.nSeg   (1,1) uint8  = 4;

        % Indices (1,2,3) of the tendons to include
        opts.usedTendons (:,1) double = [1,2,3];
    end

    link = elara.FlexibleLink;

    %% Basic Link Configuration

    link.parentLink   = 0;
    link.isCantilever = true;
    link.jointIsActuated = false;
    link.nSegments = opts.nSeg;
    link.L            = 0.5;
    link.g_J_B        = eye(4);
    link.Ba = [ eye(2); zeros(4,2) ];
    link.Bc = [ zeros(2,4); eye(4) ];
    link.xiRef = repmat([0;0;0;0;0;1], [1,link.nSegments]);
    link.beamParameters = beamParams_ASA_Round("radius", 3.5e-3);
    link.beamParameters.d = opts.d;

    %% Define Tendon Path Functions

    % Functions that define the cable path by returning the x,y coordinates
    % of the tendon location in the cross-section plane

    % Straight path
    % The path in the cross-section plane is defined in polar coordinates
    % by distance from backbone d (m) and angle alpha (deg)
    x_m_fun_straight = @(s,d,alpha) [d*cosd(alpha); d*sind(alpha); 0 ];


    %% Set Up Tendon Configuration

    % Cell array of function handles; defines the individual cable paths
    link.tendonActuation.x_td_funs = {
        @(s)x_m_fun_straight(s,0.02, 0)
        @(s)x_m_fun_straight(s,0.02, 120) 
        @(s)x_m_fun_straight(s,0.02, 240)
        };

    % Lengths at which the tendons terminate along the link length
    link.tendonActuation.LTermination = [
        link.L, link.L, link.L
        ];

    % Only use specified tendons
    link.tendonActuation.x_td_funs = link.tendonActuation.x_td_funs(opts.usedTendons);
    link.tendonActuation.LTermination = link.tendonActuation.LTermination(opts.usedTendons);

    link.tendonActuation = link.tendonActuation.getSymbolicPathDerivatives;

    %% Define TCP
    link.hasTCP = true;
    link.g_B_TCP = elara.SE3.matrix(eye(3), [0,0,0]);
end
