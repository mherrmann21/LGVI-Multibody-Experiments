%% Evaluate results of a simulation study / generate publication plots
% Visualizations etc.
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all
%addLocalPaths;

%% Specify and load results

% 0 = rigid
% 1 = contManip
% 3 = planar Manip
SYSTEM_MDL = 3;
SAVE_PLOTS = false;

% Directory where all result subfolders are
resultsDir = 'H:\Forschung\SimResults_all';
resultsDir = 'C:\Forschung\SimResults';

% Directory where all simstudy results are stored in separate subfolders
% (defined below)
plotSaveDir = 'C:\Users\ge97bij\LRZ Sync+Share\Doc\phd-thesis\plots';
plotSaveDir = 'plots';

% subfolder name
switch SYSTEM_MDL
    case 0
        subFolder = "260313_1622_simStudy_ocp_discretization__system_0";
        plotSaveSubFolder = "ocp_simstudy_labRob";
    case 1
        subFolder = "260313_1115_simStudy_ocp_discretization__system_1";
        plotSaveSubFolder = "ocp_simstudy_contManip";
    case 3
        subFolder = "260313_1107_simStudy_ocp_discretization__system_3";
        subFolder = "260716_1712_simStudy_ocp_discretization__system_3";        
        plotSaveSubFolder = "ocp_simstudy_planarManip";
    otherwise
        error("Not defined.");
end
plotSaveFolder = fullfile(plotSaveDir, plotSaveSubFolder);
% Load data
saveDir = fullfile(resultsDir, subFolder);
simStudyRes = load(fullfile(saveDir, "simStudyResults.mat"));

% Get results data
hVec = simStudyRes.hVec;
discNames = ["VI", "RK2", "RK4", "Midpoint"];
res = simStudyRes.res;

% Count "solved to acceptable level" as failed
res.success = res.return_status == "Solve_Succeeded";


% Only get successful solution data
res.qErrorM(~res.success) = nan;
res.qErrorN(~res.success) = nan;
res.fError(~res.success) = nan;
res.uError(~res.success) = nan;
res.iter_count(~res.success) = nan;
res.t_wall_total(~res.success) = nan;

% Specify which time quantity to use for computation time (CPU or wall)
res.tComp = res.t_wall_total;
%res.tComp = res.t_proc_total;

% Add convergence line for 4th-order RK4?
add4thOrderLine = (SYSTEM_MDL == 0 || SYSTEM_MDL == 3);

%% Plot settings

% Plot labels
text_qError = "$\epsilon_{\mathrm{rel},q}$";
text_uError = "$\epsilon_{\mathrm{rel},u}$";
text_fError = "$\epsilon_{{J}}$";
text_tComp = "$T_c$ in s";
text_h = "$h$ in s";

% Default for 2-plot (horizontal) layout
pdfWidth = 7.6*28.346; % width in pt

% Default value for "standard" (non-special) plots
pdfAspectRatioDefault = 1.5;

%% Visualization planar manipulator

if SYSTEM_MDL == 3
    MBSim = simStudyRes.MBSimOCPRef;
    qVis = deg2rad([-45,-90]);

    % Compute joint positions for text
    xJoints = zeros(3,length(qVis));
    gVis = MBSim.MBSys.computeFwdKin(qVis);
    for iLink = 1:MBSim.MBSys.nLinks
        gJoint = gVis(:,:,iLink)/MBSim.links(iLink).g_J_B;
        xJoints(:,iLink) = gJoint(1:3,4);
    end

    % Text offset for each joint (column for each joint)
    xOffset = [
        -0.2  0.1
        0       0
        0.1  0.15
        ];
    xJoints = xJoints + xOffset;

    % System Visualization
    colorMapFun = @(x) crameri("nuuk", x+1);
    fhVis = init3Dplot( ...
        "Name", "system visualization", ...
        "NumberTitle", "off", "Theme", "Light");
    [~, vis] = MBSim.visualizeSystemConfig( qVis,...
        "createFigure", false, "linkColorMap", colorMapFun);

    for iLink = 1:MBSim.MBSys.nLinks
        text(xJoints(1,iLink), xJoints(2,iLink), xJoints(3,iLink), ...
            sprintf("$q_%d$", iLink), "Interpreter", "latex", ...
            "FontSize", 11);
    end

    xlim([-0.3, 0.8]);
    ylim([-0.3, 0.3]);
    zlim([-0.1, 1.5]);
    axis off;

    vis.cSysI.Scale = 0.35;
    vis.cSysI.LabelFontSize = 12;
    vis.cSysI.Name = "";

    for iLink = 1:MBSim.MBSys.nLinks
        vis.linkVis(iLink).cSysJ.Visible = false;
        vis.linkVis(iLink).cSysRef.Scale = 0.2;
        vis.linkVis(iLink).cSysRef.Name = "";
    end
    view(38, 12);
    %circular_arrow(fhVis, 0.3, [0.5,-0.1], 5, 5, 1)

    if SAVE_PLOTS
        saveFigureArray(fhVis, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", 6*28.346, "pdfAspectRatio", 0.8);
    end
end


%% Visualization lab robot

if SYSTEM_MDL == 0
    MBSim = simStudyRes.MBSimOCPRef;
    qVis = deg2rad([30, -35, 70]);

    % Compute joint positions for text
    xJoints = zeros(3,length(qVis));
    gVis = MBSim.MBSys.computeFwdKin(qVis);
    for iLink = 1:MBSim.MBSys.nLinks
        gJoint = gVis(:,:,iLink)/MBSim.links(iLink).g_J_B;
        xJoints(:,iLink) = gJoint(1:3,4);
    end

    % Compute TCP position
    g_TCP = gVis(:,:,MBSim.MBSys.indexTCPFrame)*MBSim.MBSys.g_B_TCP;
    x_TCP = g_TCP(1:3,4);

    % Text offset for each joint (column for each joint)
    xOffset = [
        -0   -0.1    0.06
        -0   0 0
        -0.07  0.06 0.1
        ]*1.5;
    xJoints = xJoints + xOffset;

    % System Visualization
    colorMapFun = @(x) crameri("nuuk", x+1);
    fhVis = init3Dplot( ...
        "Name", "system visualization", ...
        "NumberTitle", "off", "Theme", "Light");
    [~, vis] = MBSim.visualizeSystemConfig( qVis,...
        "createFigure", false, "linkColorMap", colorMapFun);

    for iLink = 1:MBSim.MBSys.nLinks
        text(xJoints(1,iLink), xJoints(2,iLink), xJoints(3,iLink), ...
            sprintf("$q_%d$", iLink), "Interpreter", "latex", ...
            "FontSize", 11);
    end
    text(x_TCP(1)-0.18, x_TCP(2)-0.08, x_TCP(3)-0.02, ...
        "TCP", "Interpreter", "latex", ...
        "FontSize", 11);

    xlim([-0.14, 0.4]);
    ylim([-0.15, 0.3]);
    zlim([-0.1, 1.15]);
    axis off;

    vis.cSysI.Scale = 0.25;
    vis.cSysI.LabelFontSize = 12;
    vis.cSysI.Name = "";

    for iLink = 1:MBSim.MBSys.nLinks
        vis.linkVis(iLink).cSysJ.Visible = false;
        vis.linkVis(iLink).cSysRef.Scale = 0.15;
        vis.linkVis(iLink).cSysRef.Name = "";
    end
    vis.linkVis(end).cSysTCP.h_nameLabel.Visible = "off";
    view(45, 30);

    if SAVE_PLOTS
        saveFigureArray(fhVis, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", 10*28.346, "pdfAspectRatio", 0.9);
    end
end


%% Visualization continuum manipulator

if SYSTEM_MDL == 1
    MBSim = simStudyRes.MBSimOCPRef;
    qVis = zeros(MBSim.MBSys.nDoF,1);

    % Compute TCP position
    gVis = MBSim.MBSys.computeFwdKin(qVis);
    g_TCP = gVis(:,:,MBSim.MBSys.indexTCPFrame)*MBSim.MBSys.g_B_TCP;
    x_TCP = g_TCP(1:3,4);

    % System Visualization
    fhVis = init3Dplot( ...
        "Name", "system visualization", ...
        "NumberTitle", "off", "Theme", "Light");
    [~, vis] = MBSim.visualizeSystemConfig( qVis,...
        "createFigure", false);

    text(x_TCP(1)-0.03, x_TCP(2), x_TCP(3)+0.02, ...
        "TCP", "Interpreter", "latex", "FontSize", 11);

    xlim([-0.05, 0.2]);
    ylim([-0.05, 0.2]);
    zlim([0, 0.6]);
    axis off;

    vis.cSysI.Scale = 0.15;
    vis.cSysI.LabelFontSize = 12;
    vis.cSysI.Name = "";
    vis.cSysI.h_axisLabels(3).Position(1) = 0.06;

    vis.linkVis(1).cSysJ.Visible = false;
    vis.linkVis(1).cSysTCP.h_nameLabel.Visible = "off";
    view(135, 45);

    if SAVE_PLOTS
        saveFigureArray(fhVis, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", 5*28.346, "pdfAspectRatio", 0.6);
    end
end

%% Snapshots

if 1% SYSTEM_MDL == 3
    MBSim = simStudyRes.MBSimOCPRef;
    fhSS = init3Dplot( ...
        "Name", "snapshots", ...
        "NumberTitle", "off", "Theme", "Light");

    snapShotColormap = @(x) crameri("imola", x+1);

    if  SYSTEM_MDL == 3
        % Planar manip

        MBSim.drawSnapshots("figure", fhSS, "nSnapShots", 10, ...
            "includeColorbar", false, "snapShotColormap", snapShotColormap);
        xlim([-0.5, 1.2]);
        ylim([-0.3, 0.3]);
        zlim([-2.2, 2.2]);
        view(0,0);

        % Coordinate frame for inertial frame
        % Shift slightly in negative y direction to place text labels on top of
        % other plot stuff
        coordSysSE3(SE3Matrix(eye(3), [0,-0.15,0]), "Scale", 0.5, "Name", "", "LabelFontSize", 12, ...
            "AxisColors", repmat(lines(1), [3,1]));

        text(0.2, 0, -2, ...
            sprintf("$t = %d$ s", simStudyRes.OCPRef.tout(1)), ...
            "Interpreter", "latex", "FontSize", 11);
        text(0.2, 0, 2, ...
            sprintf("$t = %d$ s", simStudyRes.OCPRef.tout(end)), ...
            "Interpreter", "latex", "FontSize", 11);

        plotWidthSS = 3.5*28.346;
        pdfAspectRatioSS = 0.5;
    else
        % Lab robot or cont. manipulator

        MBSim.drawSnapshots("figure", fhSS, ...
            "nSnapShots", 7, "includeColorbar", false, ...
            "snapShotColormap", snapShotColormap);

        % Inertial frame
        if SYSTEM_MDL == 0
            coordSysSE3(eye(4), "Scale", 0.4, "Name", "", "LabelFontSize", 12, ...
                "AxisColors", repmat(lines(1), [3,1]));
        else
            cSysI = coordSysSE3(eye(4), "Scale", 0.15, "Name", "", "LabelFontSize", 12, ...
                "AxisColors", repmat(lines(1), [3,1]));
            cSysI.h_axisLabels(3).Position(1:2) = -0.025;
        end

        %%% Add TCP trajectory
        x_TCP_traj = simStudyRes.OCPRef.x_TCP_traj;
        colors = lines(3);
        trajColor = colors(2,:);
        % Trajectory
        plot3(x_TCP_traj(1,:), x_TCP_traj(2,:), x_TCP_traj(3,:), ...
            "LineWidth", 1.5, "Color", trajColor);
        % Markers at start and end
        plot3(x_TCP_traj(1,[1,end]), x_TCP_traj(2,[1,end]), x_TCP_traj(3,[1,end]), ...
            ".", "MarkerSize", 20, "Color", trajColor);
        % Projection on xy plane
        projColor = ones(3,1)*0.7;
        plot3(x_TCP_traj(1,:), x_TCP_traj(2,:), x_TCP_traj(3,:)*0, ...
            "LineWidth", 1.5, "Color", projColor);
        plot3(x_TCP_traj(1,[1,end]), x_TCP_traj(2,[1,end]), x_TCP_traj(3,[1,end])*0, ...
            ".", "MarkerSize", 15, "Color", projColor);

        % Text with time at start and end
        if SYSTEM_MDL == 1
            xText1 = x_TCP_traj(:,1)   + [-0.03; -0.03; 0.05];
            xText2 = x_TCP_traj(:,end) + [-0.01; -0.01; 0.05];
        else
            xText1 = x_TCP_traj(:,1)   + [-0.1; -0.03; 0.08];
            xText2 = x_TCP_traj(:,end) + [-0.1; -0.01; -0.12];
        end
        text(xText1(1), xText1(2), xText1(3), ...
            sprintf("$t = %d$ s", simStudyRes.OCPRef.tout(1)), ...
            "Interpreter", "latex", "FontSize", 11);
        text(xText2(1), xText2(2), xText2(3), ...
            sprintf("$t = %d$ s", simStudyRes.OCPRef.tout(end)), ...
            "Interpreter", "latex", "FontSize", 11);

        axis tight
        if SYSTEM_MDL == 0
            view(35,20);
            zlim([-0.05, 1.2]);
        else
            view(50,40);
        end
    end
    title("")
    axis off;
    switch SYSTEM_MDL
        case 0
            pdfAspectRatioSS = 0.8;
            plotWidthSS = 7*28.346;
        case 1
            pdfAspectRatioSS = 0.9;
            plotWidthSS = 6.5*28.346;
        case 3
            pdfAspectRatioSS = 0.9;
            plotWidthSS = 7*28.346;
    end

    if SAVE_PLOTS
        saveFigureArray(fhSS, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", plotWidthSS, "pdfAspectRatio", pdfAspectRatioSS);
    end
end


%% Coordinates and inputs

plotLineWidth = 1.2;
plotData = {simStudyRes.q_ref, simStudyRes.u_ref};

% Colors for plots with 3 components
colors3 = [
    tumColors().TUMBlue4;
    tumColors().TUMBlue2;
    tumColors().TUMOrange;
    ];


OCPRef = simStudyRes.OCPRef;
for iC = 1%:2
    plotDof = 1:size(plotData{iC},1); % which qs to plot as joint angles
    fhs_jointAngles = figure(...
        "Name", sprintf("ref_sol_data_%d", iC), ...
        "NumberTitle", "off", "Theme", "Light");

    tl = tiledlayout("vertical", "TileSpacing", "tight", "Padding", "tight");
    ax = nexttile;

    ph = plot(simStudyRes.OCPRef.tout, ...
        plotData{iC}, "LineWidth", plotLineWidth);

    % Add markers to indicate B-spline control points
    if iC == 2 && OCPRef.useSplineInputs
        % Get Greville abscissae points as time values for the control
        % points (average time value of the time span influenced by a
        % control point)
        [~,~,~,tCP] = OCPRef.getInputSplineBasisMatrix;
        % Get data value at Greville points
        uCP = interp1(OCPRef.tout, simStudyRes.u_ref.', tCP.');

        hold on;
        plot(tCP, uCP, "o", "HandleVisibility", "off", ...
            "MarkerSize", 3.5);
    end
    grid on;
    ax.TickLabelInterpreter = "latex";
    axis padded;
    xlim(MBSim.simRes.tout([1,end]));
    if SYSTEM_MDL == 1 && iC == 1
        % colors = repmat(crameri('roma',MBSim.MBSys.nDoF), [3,1]);
        colors = tumBlueMap(MBSim.MBSys.nDoF);
    else
        % colors = repmat(crameri('romaO',4+1), [3,1]);
        colors = colors3;
    end
    colororder(colors(1:size(plotData{iC},1),:));

    if iC == 1
        switch SYSTEM_MDL
            case 0
                ylabel("$q$ in rad", "Interpreter", "latex");
                legend(arrayfun(@(x) sprintf("$q_{%d}$", x), plotDof), ...
                    "interpreter", "latex", "Location", "southwest", ...
                    "IconColumnWidth", 15, "Orientation", "horizontal");
            case 1
                ylabel("$q$", "Interpreter", "latex");    

                % Invisible proxy for the legend entry
                hold on;
                hProxy = plot(ax, NaN, NaN, ...
                    'LineStyle', 'none', ...
                    'DisplayName', '$\dots$');
                legend([ph(1),hProxy,ph(end)], ["$q_1$", "$\vdots$", "$q_8$"], ...
                    "interpreter", "latex", "Location", "east", ...
                    "IconColumnWidth", 15);
            case 3
                ylabel("$q$ in rad", "Interpreter", "latex");
                legend(arrayfun(@(x) sprintf("$q_{%d}$", x), plotDof), ...
                    "interpreter", "latex", "Location", "northeast", ...
                    "IconColumnWidth", 15, "Orientation", "horizontal");
        end
    else
        if SYSTEM_MDL  ~= 1
            ylabel("$u$ in Nm", "Interpreter", "latex");
        else
            ylabel("$u$ in N", "Interpreter", "latex");
        end
        legend(arrayfun(@(x) sprintf("$u_{%d}$", x), plotDof), ...
            "interpreter", "latex", "Location", "northwest", ...
            "IconColumnWidth", 15, "Orientation", "horizontal");
    end

    %%% Derivatives
    data_dt = diff2ndOrder(plotData{iC}, simStudyRes.hRef);
    ax = nexttile;
    plot(simStudyRes.OCPRef.tout, data_dt, "LineWidth", plotLineWidth);
    grid on;
    if iC == 1
        if SYSTEM_MDL  ~= 1
            ylabel("$\dot{q}$ in rad/s", "Interpreter", "latex");
        else
            ylabel("$\dot{q}$ in 1/s", "Interpreter", "latex");
        end
    else
        if SYSTEM_MDL  ~= 1
            ylabel("$\dot{u}$ in Nm/s", "Interpreter", "latex");
        else
            ylabel("$\dot{u}$ in N/s", "Interpreter", "latex");
        end
    end
    xlabel("time $t$ in s", "Interpreter", "latex");
    ax.TickLabelInterpreter = "latex";
    axis padded;
    xlim(MBSim.simRes.tout([1,end]));
    if SYSTEM_MDL == 0
        %ylim([-30, 25]);
    end

    if SAVE_PLOTS
        drawnow;
        pdfWidth = 7.5*28.346; % width in pt
        pdfAspectRatio = 1.2;

        saveFigureArray(fhs_jointAngles, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatio);
    end
end

%% TCP Trajectory Lab Robot / Cont. Manip.

if SYSTEM_MDL ~= 3
    OCPRef = simStudyRes.OCPRef;
    MBSim = simStudyRes.MBSimOCPRef;

    % Compute Ref. actual TCP trajectory
    g_TCP_sol = zeros(4,4, OCPRef.nSteps+1);
    for iStep = 1:OCPRef.nSteps+1
        g_k = MBSim.MBSys.computeFwdKin(simStudyRes.q_ref(:,iStep));
        g_TCP_sol(:,:,iStep) = g_k(:,:,MBSim.MBSys.indexTCPFrame)*MBSim.MBSys.g_B_TCP;
    end
    x_TCP_sol = squeeze(g_TCP_sol(1:3,4,:));
    x_dot_TCP_sol = diff2ndOrder(x_TCP_sol, OCPRef.h);
    x_dot_TCP_des = diff2ndOrder(OCPRef.x_TCP_traj, OCPRef.h);

    dataSol = {x_TCP_sol,         x_dot_TCP_sol};
    dataDes = {OCPRef.x_TCP_traj, x_dot_TCP_des};
    axisStrings = {
        ["$x$", "$y$", "$z$"] + " in m"
        ["$\dot{x}$", "$\dot{y}$", "$\dot{z}$"] + " in m/s"
        };

    fhs_traj = gobjects(2,1);
    for iData = 1:2
        fhs_traj(iData) = figure(...
            "Name", sprintf("tcp_trajectory data %d", iData), ...
            "NumberTitle", "off", "Theme", "Light");
        tl = tiledlayout("vertical", ...
            "TileSpacing", "tight", "Padding", "tight");
        for iAxis = 1:3
            ax = nexttile;
            plot(OCPRef.tout, dataDes{iData}(iAxis,:), "-", ...
                "LineWidth", plotLineWidth);
            hold on;
            plot(OCPRef.tout, dataSol{iData}(iAxis,:), "-.", ...
                "LineWidth", plotLineWidth);
            grid on;
            axis padded;
            xlim(OCPRef.tout([1,end]));
            ylabel(axisStrings{iData}(iAxis), "Interpreter", "latex");
            ax.TickLabelInterpreter = "latex";
            if iData == 1 && iAxis == 1
                legend("desired", "ref. solution", ...
                    "interpreter", "latex", "Location", "northeast", ...
                    "IconColumnWidth", 25);
            end
            %colororder([tumColors().TUMBlue; tumColors().TUMOrange]);
        end
        xlabel("time $t$ in s", "Interpreter", "latex");

    end
    if SAVE_PLOTS
        pdfWidth = 7.5*28.346; % width in pt
        pdfAspectRatio = 1.2;

        saveFigureArray(fhs_traj, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatio);
    end
end
