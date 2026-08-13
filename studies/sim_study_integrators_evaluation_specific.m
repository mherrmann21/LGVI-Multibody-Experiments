%% Evaluate Integrator Simulation Study
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all


%% Specify and load results

% 0 = Pendulum
% 1 = Cantilever beam HK24
% 2 = Rigid-Flexible Robot
SYSTEM_MDL = 2;

SAVE_PLOTS = true;

% Directory where all result subfolders are
resultsDir = fullfile(getRootFolder, "results", "runs");

% Directory where all simstudy results are stored in separate subfolders
% (defined below)
plotSaveDir = fullfile(getRootFolder, "results", "plots", "time-integration");

% Subfolder names
switch SYSTEM_MDL
    case 0
        subFolder(1) = "260813_1104_simStudy_integrators__system_0_dissip_0";
        subFolder(2) = "260813_1106_simStudy_integrators__system_0_dissip_1";
        plotSaveSubFolder = "integrator_simstudy_rigid";
    case 1
        subFolder(1) = "260813_1110_simStudy_integrators__system_1_dissip_0";
        subFolder(2) = "260813_1132_simStudy_integrators__system_1_dissip_1";
        plotSaveSubFolder = "integrator_simstudy_flexible";
    case 2
        subFolder = "260813_1136_simStudy_integrators__system_2_dissip_1";
        plotSaveSubFolder = "integrator_simstudy_combined";
    otherwise
        error("Not defined.");
end

% Load data
nCases = length(subFolder);
for iC = 1:nCases
    simStudyRes(iC) = load(fullfile(resultsDir, subFolder(iC), "simStudyResults.mat"));
end

if SAVE_PLOTS
    plotSaveFolder = fullfile(plotSaveDir, plotSaveSubFolder);
    if ~isfolder( plotSaveFolder )
        mkdir( plotSaveFolder );
    end
end


%% System Visualization Rigid

if SYSTEM_MDL == 0
    iC = 1;
    qVis = deg2rad([45;-15;-15;-15]);

    % Compute joint positions for text
    xJoints = zeros(3,4);
    gVis = simStudyRes(iC).MBSim.system.computeFwdKin(qVis);
    for iLink = 1:simStudyRes(iC).MBSim.system.nLinks
        gJoint = gVis(:,:,iLink)/simStudyRes(iC).MBSim.links(iLink).g_J_B;
        xJoints(:,iLink) = gJoint(1:3,4);
    end

    % Text offset for each joint (column for each joint)
    xOffset = [
        -0.15  -0.15 -0.1  -0.1
        0     0    0      0
        -0.15 -0.15 -0.2   -0.15
        ];
    xJoints = xJoints + xOffset;

    % System Visualization
    fhVis = elara.visualization.initializeAxes( ...
        "Name", sprintf("system visualization case %d", iC), ...
        "NumberTitle", "off", "Theme", "Light");

    colorMapFun = @(x) crameri("nuuk", x+1);
    [~, vis] = simStudyRes(iC).MBSim.visualizeSystemConfig( qVis,...
        "createFigure", false, "linkColorMap", colorMapFun);

    for iLink = 1:simStudyRes(iC).MBSim.system.nLinks
        text(xJoints(1,iLink), xJoints(2,iLink), xJoints(3,iLink), ...
            sprintf("$q_%d$", iLink), "Interpreter", "latex", ...
            "FontSize", 11);
    end

    xlim([-0.1, 3.6]);
    ylim([-0.3, 0.3]);
    zlim([-1.6, 0.35]);
    axis off;

    vis.coordSysI.Scale = 0.3;
    vis.coordSysI.LabelFontSize = 12;
    vis.coordSysI.Name = "";

    for iLink = 1:simStudyRes(iC).MBSim.system.nLinks
        vis.linkVisualization(iLink).coordSysJ.Visible = false;
        vis.linkVisualization(iLink).coordSysRef.Scale = 0.2;
        vis.linkVisualization(iLink).coordSysRef.Name = "";
    end
    view(38, 12);
    %circular_arrow(fhVis, 0.3, [0.5,-0.1], 5, 5, 1)

    if SAVE_PLOTS
        saveFigureArray(fhVis, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", false, "savePDF", true, ...
            "pdfWidth", 10*28.346, "pdfAspectRatio", 1.5);
    end
end


%% System Visualization Rigid-Flex

if SYSTEM_MDL == 2
    iC = 1;
    for iConf = 1:2
        if iConf == 1
            qVis = simStudyRes(iC).MBSim.system.qRef*0;
        else
            qVis = simStudyRes(iC).MBSim.system.qRef;
        end

        % Compute joint positions for text
        xJoints = zeros(3,4);
        gVis = simStudyRes(iC).MBSim.system.computeFwdKin(qVis);
        for iLink = 1:simStudyRes(iC).MBSim.system.nLinks
            gJoint = gVis(:,:,iLink)/simStudyRes(iC).MBSim.links(iLink).g_J_B;
            xJoints(:,iLink) = gJoint(1:3,4);
        end

        % Text offset for each joint (column for each joint)
        if iConf == 1
            xOffset = [
                -0.15  -0.14 -0.12   -0.12
                0      0      0       0
                0      0.01   0       0
                ];
        else
            xOffset = [
                -0.18 -0.15   0.04   -0.08
                0     0       0      0
                0     0.05    0.08   0.05
                ];
        end
        xJoints = xJoints + xOffset*1.3;

        % System Visualization
        colorMapFun = @(x) crameri("nuuk", x+1);
        fhVis = elara.visualization.initializeAxes( ...
            "Name", sprintf("system visualization conf %d", iConf), ...
            "NumberTitle", "off", "Theme", "Light");
        [~, vis] = simStudyRes(iC).MBSim.visualizeSystemConfig( qVis,...
            "createFigure", false, "linkColorMap", colorMapFun);

        for iLink = 1:simStudyRes(iC).MBSim.system.nLinks
            text(xJoints(1,iLink), xJoints(2,iLink), xJoints(3,iLink), ...
                sprintf("$q_%d$", iLink), "Interpreter", "latex", ...
                "FontSize", 11);
        end

        if iConf == 1
            xlim([-0.2, 0.2]);
            ylim([-0.2, 0.25]);
            zlim([0, 1.5]);
        else
            xlim([-0.3, 0.4]);
            ylim([-0.3, 0.3]);
            zlim([0, 0.9]);
        end
        axis off;

        vis.coordSysI.Scale = 0.15;
        vis.coordSysI.LabelFontSize = 12;
        vis.coordSysI.Name = "";

        for iLink = 1:3
            vis.linkVisualization(iLink).coordSysJ.Visible = false;
            vis.linkVisualization(iLink).coordSysRef.Scale = 0.1;
            vis.linkVisualization(iLink).coordSysRef.Name = "";
        end
        vis.linkVisualization(end).coordSysJ.Visible = false;
        vis.linkVisualization(end).coordSysTCP.Visible = false;
        view(12, 22);
        %circular_arrow(fhVis, 0.3, [0.5,-0.1], 5, 5, 1)

        if SAVE_PLOTS
            if iConf == 1
                pdfWidthV = 4*28.346;
                pdfAspectRatioV = 0.4;
            else
                pdfWidthV = 6.5*28.346;
                pdfAspectRatioV = 0.9;
            end

            saveFigureArray(fhVis, plotSaveFolder, ...
                "saveFig", true, "saveJPEG", false, "savePDF", true, ...
                "pdfWidth", pdfWidthV, "pdfAspectRatio", pdfAspectRatioV);
        end
    end
end


%% Snapshots

if SYSTEM_MDL == 0
    for iC = 1:nCases
        fhSS = elara.visualization.initializeAxes( ...
            "Name", sprintf("snapshots case %d", iC), ...
            "NumberTitle", "off", "Theme", "Light");

        colorMapFun = @(x) crameri("imola", x+1);
        simStudyRes(iC).MBSimRef.drawSnapshots("figure", fhSS, ...
            "nSnapShots", 15, "includeColorbar", false, ...
            "snapShotColormap", colorMapFun);
        title("")
        xlim([-4.1, 4.1]);
        ylim([-0.3, 0.3]);
        zlim([-3.6, 0.45]);
        view(0,0);

        axis off;

        % Coordinate frame for inertial frame
        % Shift slightly in negative y direction to place text labels on top of
        % other plot stuff
        elara.visualization.CoordinateFrame(elara.SE3.matrix(eye(3), ...
            [0,-0.15,0]), "Scale", 0.5, "Name", "", "LabelFontSize", 12, ...
            "AxisColors", repmat(lines(1), [3,1]));

        if SAVE_PLOTS
            saveFigureArray(fhSS, plotSaveFolder, ...
                "saveFig", true, "saveJPEG", false, "savePDF", true, ...
                "pdfWidth", 8*28.346, "pdfAspectRatio", 1.7);
        end
    end
end

%% Joint angles and velocities

plotLineWidth = 1.2;

if SYSTEM_MDL ~= 1
    plotDof = 1:4; % which qs to plot as joint angles
    for iC = 1:nCases

        fhs_jointAngles = figure(...
            "Name", sprintf("joint angles case %d", iC), ...
            "NumberTitle", "off", "Theme", "Light");

        tl = tiledlayout("vertical", "TileSpacing", "tight", "Padding", "tight");
        ax = nexttile;

        plot(simStudyRes(iC).MBSimRef.results.tout, ...
            simStudyRes(iC).MBSimRef.results.q(plotDof,:), "LineWidth", plotLineWidth);
        grid on;
        ylabel("$q$ in rad", "Interpreter", "latex");
        %xlabel("time $t$ in s", "Interpreter", "latex");
        ax.TickLabelInterpreter = "latex";
        %xlim([OCP.tout(1), OCP.tout(end)]);
        %colororder(ax, qColors);
        axis padded;
        xlim(simStudyRes(iC).MBSimRef.results.tout([1,end]));
        if SYSTEM_MDL == 0
            ylim([-15, 5]);
        end
        colororder(crameri('romaO',4+1));

        switch SYSTEM_MDL
            case 0
                legendPos = "southwest";
            case 2
                legendPos = "east";
            otherwise
                legendPos = "best";
        end
        if iC == 1
            legend(arrayfun(@(x) sprintf("$q_{%d}$", x), plotDof), ...
                "interpreter", "latex", "Location", legendPos, ...
                "IconColumnWidth", 15, "Orientation", "horizontal");
        end

        ax = nexttile;
        plot(simStudyRes(iC).MBSimRef.results.tout, ...
            simStudyRes(iC).MBSimRef.results.q_dot(plotDof,:), "LineWidth", plotLineWidth);
        grid on;
        ylabel("$\dot{q}$ in rad/s", "Interpreter", "latex");
        xlabel("time $t$ in s", "Interpreter", "latex");
        ax.TickLabelInterpreter = "latex";
        % xlim([OCP.tout(1), OCP.tout(end)]);
        % colororder(ax, qColors);
        axis padded;
        xlim(simStudyRes(iC).MBSimRef.results.tout([1,end]));
        if SYSTEM_MDL == 0
            ylim([-30, 25]);
        end

        if SAVE_PLOTS
            drawnow;
            pdfWidth = 7.5*28.346; % width in pt
            pdfAspectRatio = 1.15;

            saveFigureArray(fhs_jointAngles, plotSaveFolder, ...
                "saveFig", true, "saveJPEG", false, "savePDF", true, ...
                "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatio);
        end
    end
end

%% Discrete deformations

if SYSTEM_MDL == 2
    nSeg = simStudyRes(iC).MBSimRef.links(end).nSegments;
    dofPsiX = 5:2:3+nSeg*2;
    dofPsiY = 6:2:4+nSeg*2;

    for iC = 1:nCases

        fhs_jointAngles = figure(...
            "Name", sprintf("deformations case %d", iC), ...
            "NumberTitle", "off", "Theme", "Light");

        tl = tiledlayout("vertical", "TileSpacing", "tight", "Padding", "tight");
        ax = nexttile;

        ph = plot(simStudyRes(iC).MBSimRef.results.tout, ...
            simStudyRes(iC).MBSimRef.results.q(dofPsiX,:), "LineWidth", plotLineWidth);
        grid on;
        ylabel("$\psi_x$", "Interpreter", "latex");
        %xlabel("time $t$ in s", "Interpreter", "latex");
        ax.TickLabelInterpreter = "latex";
        %xlim([OCP.tout(1), OCP.tout(end)]);
        %colororder(ax, qColors);
        axis padded;
        xlim(simStudyRes(iC).MBSimRef.results.tout([1,end]));
        if SYSTEM_MDL == 0
            ylim([-15, 5]);
        end
        %colororder(crameri('romaO',nSeg+1));
        colororder(tumBlueMap(nSeg));


        legendDof = round(linspace(1,nSeg,4));
        legend(ph(legendDof), arrayfun(@(x) sprintf("seg. %d", x), legendDof), ...
            "interpreter", "latex", "Location", "east", ...
            "IconColumnWidth", 15, "BackgroundAlpha", 0.85);

        % if iC == 0
        %     legend(arrayfun(@(x) sprintf("$q_{%d}$", x), plotDof), ...
        %         "interpreter", "latex", "Location", "southwest", ...
        %         "IconColumnWidth", 15, "Orientation", "horizontal");
        % end

        ax = nexttile;
        plot(simStudyRes(iC).MBSimRef.results.tout, ...
            simStudyRes(iC).MBSimRef.results.q(dofPsiY,:), "LineWidth", plotLineWidth);
        grid on;
        ylabel("$\psi_y$", "Interpreter", "latex");
        xlabel("time $t$ in s", "Interpreter", "latex");
        ax.TickLabelInterpreter = "latex";
        % xlim([OCP.tout(1), OCP.tout(end)]);
        % colororder(ax, qColors);
        axis padded;
        xlim(simStudyRes(iC).MBSimRef.results.tout([1,end]));
        if SYSTEM_MDL == 0
            ylim([-30, 25]);
        end

        if SAVE_PLOTS
            drawnow;
            pdfWidth = 7.5*28.346; % width in pt
            pdfAspectRatio = 1.15;

            saveFigureArray(fhs_jointAngles, plotSaveFolder, ...
                "saveFig", true, "saveJPEG", false, "savePDF", true, ...
                "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatio);
        end
    end
end

%% End script
disp("Finished.");
