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
SYSTEM_MDL = 0;

SAVE_PLOTS = true;

% Directory where all result subfolders are
resultsDir = fullfile(getRootFolder, "results", "runs");

% Directory where all simstudy results are stored in separate subfolders
% (defined below)
plotSaveDir = fullfile(getRootFolder, "results", "plots", "time-integration");

% Subfolder names
switch SYSTEM_MDL
    case 0
        %subFolder(1) = "260212_1211_simStudy_integrators__system_0_dissip_0";
        %subFolder(2) = "260212_1157_simStudy_integrators__system_0_dissip_1";
        subFolder(1) = "260719_1835_simStudy_integrators__system_0_dissip_0";
        subFolder(2) = "260719_1838_simStudy_integrators__system_0_dissip_1";
        plotSaveSubFolder = "integrator_simstudy_rigid";
    case 1
        % subFolder(1) = "260212_1455_simStudy_integrators__system_1_dissip_0";
        % subFolder(2) = "260212_1554_simStudy_integrators__system_1_dissip_1";
        subFolder(1) = "260716_1631_simStudy_integrators__system_1_dissip_0";
        subFolder(2) = "260716_1631_simStudy_integrators__system_1_dissip_1";
        plotSaveSubFolder = "integrator_simstudy_flexible";
    case 2
        %subFolder = "260224_1428_simStudy_integrators__system_2_dissip_1";
        subFolder = "260716_1634_simStudy_integrators__system_2_dissip_1";
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


%% Plot settings

% Default for 2-plot (horizontal) layout
pdfWidth = 7.6*28.346; % width in pt

% Default value for "standard" (non-special) plots
pdfAspectRatioDefault = 1.5;

% Drawing options
plotLineWidth = 1.6;
plotMarkerSize = 3;
lineWidthConv = 0.8;

pOpts = struct();
switch SYSTEM_MDL
    case 0
        % All integrator names (from simstudy, with dissipation and without)
        % Used to match the individual settings to the results
        pOpts.intNames = ["VI-T", "VI-R", "ode23", "ode113", "ode45", "ode78", "ode89", "RADAU", "CVODE-N"];

        % Specify which integrators to include in the plot (sorted according to intNames)
        pOpts.includeInt = [1,1,0,1,1,1,1,0,1];
    case 1
        pOpts.intNames = ["VI-T", "VI-R", "ode15s", "ode23t", "RADAU", "CVODE-S"];
        pOpts.includeInt = [1,1,1,1,1,1];
    case 2
        pOpts.intNames = ["VI-T", "VI-R", "ode15s", "ode23t", "RADAU", "CVODE-S"];
        pOpts.includeInt = [1,1,1,1,1,1];
end

% Specify fixed color and line style for each integrator (sorted according to intNames)
pOpts.intColors = zeros(length(pOpts.intNames),3);
%cols = crameri('imola', sum(pOpts.includeInt == 1)+1);% Looks great but unfortunately too similar
cols = crameri('-romaO', sum(pOpts.includeInt == 1)+1);
pOpts.intColors(pOpts.includeInt == 1, :) = cols(1:sum(pOpts.includeInt == 1),:);

allLineStyles = [ "-", "-.", "--", "-", "-.", "--", "-."];
allMarkers    = ["o", "square", "+", "o", "square", "+", "o"];
pOpts.lineStyles = strings(length(pOpts.intNames),1);
pOpts.lineStyles(pOpts.includeInt == 1) = allLineStyles(1:sum(pOpts.includeInt == 1));
pOpts.lineMarkers = strings(length(pOpts.intNames),1);
pOpts.lineMarkers(pOpts.includeInt == 1) = allMarkers(1:sum(pOpts.includeInt == 1));

% Axis labels
label_tComp = "$T_{\mathrm{c}}$ in s";
label_qError = "$\epsilon_{\mathrm{rel}}$";
label_hMean  = "$h_{\mathrm{mean}}$ in s";
label_params = ["$h$ in s", "abstol"];


%% Integration Error over Parameter (time step for VIs/ tolerance for odes)

for iC = 1:nCases

    fh_qError_h   =  figure(...
        "Name", sprintf("qError-h case %d", iC), ...
        "NumberTitle", "off", "Theme", "Light");
    ax_qError_h   = axes(fh_qError_h);

    fh_qError_tol =  figure(...
        "Name", sprintf("qError-tol case %d", iC), ...
        "NumberTitle", "off", "Theme", "Light");
    ax_qError_tol = axes(fh_qError_tol);

    % Add convergence line to VI plot
    switch SYSTEM_MDL
        case 0
            xVals = simStudyRes(iC).intDef(1).ParamVec;
            yVals = simStudyRes(iC).res(1).qErrorNorm;
            coeffFactor = [1.05, 1.7];
            ihCoeff = 2;
            coeff2O = yVals(ihCoeff,1) / xVals(ihCoeff)^2 * coeffFactor(iC);
            loglog(ax_qError_h, xVals,  (coeff2O * xVals.^2), "k--", "LineWidth", lineWidthConv, ...
                "DisplayName", sprintf("$%.1f h^{%.0f}$", coeff2O, 2));

        case 1
            % Add first and second-order conv lines
            ihCoeff = 2;
            xValsAll = 2.^(-14:-0.5:-18.5);

            if iC == 2
                xVals1O = simStudyRes(iC).intDef(2).ParamVec;
                yVals1O = simStudyRes(iC).res(2).qErrorNorm;
                coeff1O = yVals1O(ihCoeff,1) / xVals1O(ihCoeff)^1 * 1.1;
                loglog(ax_qError_h, xValsAll,  (coeff1O * xValsAll.^1), "k--", ...
                    "LineWidth", lineWidthConv, ...
                    "DisplayName", sprintf("$%.1f h^{%.0f}$", coeff1O, 1));
                hold(ax_qError_h, "on");
            end
            xVals2O = simStudyRes(iC).intDef(1).ParamVec;
            yVals2O = simStudyRes(iC).res(1).qErrorNorm;
            coeff2O = yVals2O(ihCoeff,1) / xVals2O(ihCoeff)^2 * 1.1;
            loglog(ax_qError_h, xValsAll,  (coeff2O * xValsAll.^2), "k-.", ...
                "LineWidth", lineWidthConv, ...
                "DisplayName", sprintf("$%.1f h^{%.0f}$", coeff2O, 2));
        case 2
            % Add first and second-order conv lines
            ihCoeff = 2;
            xValsAll = 2.^(-9:-0.5:-15);
            xVals1O = simStudyRes(iC).intDef(2).ParamVec;
            yVals1O = simStudyRes(iC).res(2).qErrorNorm;
            coeff1O = yVals1O(ihCoeff,1) / xVals1O(ihCoeff)^1 * 0.9;
            loglog(ax_qError_h, xValsAll,  (coeff1O * xValsAll.^1), "k--", ...
                "LineWidth", lineWidthConv, ...
                "DisplayName", sprintf("$%.1f h^{%.0f}$", coeff1O, 1));
            hold(ax_qError_h, "on");
            xVals2O = simStudyRes(iC).intDef(1).ParamVec;
            yVals2O = simStudyRes(iC).res(1).qErrorNorm;
            coeff2O = yVals2O(ihCoeff,1) / xVals2O(ihCoeff)^2 * 1.05;
            loglog(ax_qError_h, xValsAll,  (coeff2O * xValsAll.^2), "k-.", ...
                "LineWidth", lineWidthConv, ...
                "DisplayName", sprintf("$%.1f h^{%.0f}$", coeff2O, 2));
    end

    for iInt = 1:length(simStudyRes(iC).res)
        intID = find(pOpts.intNames == simStudyRes(iC).intDef(iInt).Name);
        if contains(simStudyRes(iC).intDef(iInt).Name, "VI")
            ax = ax_qError_h;
        else
            ax = ax_qError_tol;
        end
        if pOpts.includeInt(intID)
            hold(ax, "on");
            loglog(ax, simStudyRes(iC).intDef(iInt).ParamVec, ...
                simStudyRes(iC).res(iInt).qErrorNorm, "-o", ...
                "DisplayName", simStudyRes(iC).intDef(iInt).Name, ...
                "Color", pOpts.intColors(intID, :), ...
                "LineWidth", plotLineWidth, ...
                "MarkerSize", plotMarkerSize, ...
                "LineStyle", pOpts.lineStyles(intID), ...
                "Marker", pOpts.lineMarkers(intID)...
                );
        end
    end

    allFigs = [fh_qError_h, fh_qError_tol];
    allAxes = [ax_qError_h, ax_qError_tol ];

    for iFig = 1:2
        figure(allFigs(iFig));
        grid on;
        xlabel(label_params(iFig), "Interpreter", "latex");
        ylabel(label_qError, "Interpreter", "latex");
        box on;
        switch SYSTEM_MDL
            case 0
                if ~(iFig == 2 && iC == 2)
                    legend("Interpreter", "latex", ...
                        "IconColumnWidth", 30, ...
                        "Location", "northwest");
                end
            case 1
                if ~(iFig == 2 && iC == 1)
                    legend("Interpreter", "latex", ...
                        "IconColumnWidth", 20, ...
                        "Location", "southeast", "BackgroundAlpha", 0.85);
                end
            case 2
                legend("Interpreter", "latex", ...
                    "IconColumnWidth", 20, ...
                    "Location", "southeast", "BackgroundAlpha", 0.85);
        end
        axis tight;

        allAxes(iFig).XScale = "log";
        allAxes(iFig).YScale = "log";
        allAxes(iFig).TickLabelInterpreter = "latex";
    end

    % Axis settings
    switch SYSTEM_MDL
        case 0
            ax_qError_h.YLim    = [1.5e-5, 20];
            ax_qError_h.YTick   = [1e-4, 1e-2, 1];
            ax_qError_tol.YLim  = [1.5e-5, 10];
            ax_qError_tol.YTick = [1e-4, 1e-2, 1];
        case 1
            ax_qError_h.XTick   = [5e-6, 1e-5, 5e-5];
            ax_qError_h.XLim    = xValsAll([end,1]);
            ax_qError_h.XTickLabel = ["$5\cdot10^{-6}$", "$10^{-5}$", "$5\cdot10^{-5}$"];
            ax_qError_h.YLim    = [5e-9, 5e-5];
            ax_qError_h.YTick   = [1e-8, 1e-7, 1e-6, 1e-5];

            ax_qError_tol.XLim  = [1e-11, 1e-2];
            ax_qError_tol.XTick = flip([1e-3, 1e-5, 1e-7, 1e-9, 1e-11]);

            if iC == 1
                ax_qError_tol.YLim  = [5e-7, 3e-2];
                ax_qError_tol.YTick = [1e-6, 1e-5, 1e-4, 1e-3, 1e-2];
            else
                ax_qError_tol.YLim  = [1e-8, 0.055];
                ax_qError_tol.YTick = [1e-8, 1e-6, 1e-4, 1e-2, 1e-0];
            end

        case 2
            ax_qError_h.YTick   = [1e-6, 1e-4, 1e-2];
            ax_qError_tol.XTick = 10.^(-11:2:-3);
            ax_qError_tol.YTick = [1e-5, 1e-3, 1e-1];
    end

    if SAVE_PLOTS
        saveFigureArray(allFigs, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", false, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatioDefault, ...
            "outsideLegendMatchWidth", false, ...
            "outsideLegendYShift", 0);
    end
end

%% Computation time over parameter

for iC = 1:nCases

    fh_tComp_h   =  figure(...
        "Name", sprintf("tComp-h case %d", iC), ...
        "NumberTitle", "off", "Theme", "Light");
    ax_tComp_h   = axes(fh_tComp_h);

    fh_tComp_tol =  figure(...
        "Name", sprintf("tComp-tol case %d", iC), ...
        "NumberTitle", "off", "Theme", "Light");
    ax_tComp_tol = axes(fh_tComp_tol);

    for iInt = 1:length(simStudyRes(iC).res)
        intID = find(pOpts.intNames == simStudyRes(iC).intDef(iInt).Name);
        if contains(simStudyRes(iC).intDef(iInt).Name, "VI")
            ax = ax_tComp_h;
        else
            ax = ax_tComp_tol;
        end
        if pOpts.includeInt(intID)
            loglog(ax, simStudyRes(iC).intDef(iInt).ParamVec, ...
                simStudyRes(iC).res(iInt).tComp, "-o", ...
                "DisplayName", simStudyRes(iC).intDef(iInt).Name, ...
                "Color", pOpts.intColors(intID, :), ...
                "LineWidth", plotLineWidth, ...
                "MarkerSize", plotMarkerSize, ...
                "LineStyle", pOpts.lineStyles(intID), ...
                "Marker", pOpts.lineMarkers(intID)...
                );
            hold(ax, "on");
        end
    end

    allFigs = [fh_tComp_h, fh_tComp_tol];
    allAxes = [ax_tComp_h, ax_tComp_tol ];

    for iFig = 1:2
        figure(allFigs(iFig));
        grid on;
        xlabel(label_params(iFig), "Interpreter", "latex");
        ylabel(label_tComp, "Interpreter", "latex");
        box on;
        switch SYSTEM_MDL
            case 0
                if ~(iFig == 2 && iC == 2) && ~(iFig == 1 && iC == 1 )
                    legend("Interpreter", "latex", "IconColumnWidth", 25, ...
                        "Location", "southwest", "BackgroundAlpha", 0.85);
                end
            case 1
                if iC ~= 1
                    legend("Interpreter", "latex", "IconColumnWidth", 25, ...
                        "Location", "southwest");
                end
            case 2
                legend("Interpreter", "latex", "IconColumnWidth", 25, ...
                    "Location", "southwest");
        end
        axis tight;

        allAxes(iFig).XScale = "log";
        allAxes(iFig).YScale = "log";
        allAxes(iFig).TickLabelInterpreter = "latex";
    end

    % Axis settings
    switch SYSTEM_MDL
        case 0
        case 1
            ax_tComp_h.XTick   = [5e-6, 1e-5, 5e-5];
            ax_tComp_h.XLim    = xValsAll([end,1]);
            ax_tComp_h.XTickLabel = ["$5\cdot10^{-6}$", "$10^{-5}$", "$5\cdot10^{-5}$"];
            ax_tComp_h.YLim    = [1.5e-1, 4];
            ax_tComp_h.YTick   = [0.3, 0.5, 1, 2, 3];

            ax_tComp_tol.XLim  = [1e-11, 1e-2];
            %ax_tComp_tol.YLim  = [1e-1, 4];
            ax_tComp_tol.XTick = flip([1e-3, 1e-5, 1e-7, 1e-9, 1e-11]);
            %ax_tComp_tol.YTick = [1e-6, 1e-5, 1e-4, 1e-3];
            if iC == 2
                ax_tComp_tol.YLim  = [1e-1, 4];
            end
        case 2
            ax_tComp_tol.XTick = 10.^(-11:2:-3);
            ax_tComp_tol.YLim  = [1e-1, 1e1];
            %ax_tComp_tol.YTick = [1e-5, 1e-3, 1e-1];
    end


    % ax_tComp_h.YTick = [1e-4, 1e-2, 1];
    % ax_tComp_tol.YLim = [1.5e-5, 10];
    % ax_tComp_tol.YTick = [1e-4, 1e-2, 1];

    if SAVE_PLOTS
        saveFigureArray(allFigs, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", false, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatioDefault, ...
            "outsideLegendMatchWidth", false, ...
            "outsideLegendYShift", 0);
    end
end

%% Computation Time over Integration Error

for iC = 1:nCases

    fh_tComp_qError = figure(...
        "Name", sprintf("tComp-qError case %d", iC), ...
        "NumberTitle", "off", "Theme", "Light");

    for iInt = 1:length(simStudyRes(iC).res)
        intID = find(pOpts.intNames == simStudyRes(iC).intDef(iInt).Name);
        if pOpts.includeInt(intID)
            loglog(simStudyRes(iC).res(iInt).qErrorNorm, ...
                simStudyRes(iC).res(iInt).tComp, ...
                "DisplayName", simStudyRes(iC).intDef(iInt).Name, ...
                "Color", pOpts.intColors(intID, :), ...
                "LineWidth", plotLineWidth, ...
                "MarkerSize", plotMarkerSize, ...
                "LineStyle", pOpts.lineStyles(intID), ...
                "Marker", pOpts.lineMarkers(intID)...
                );
            hold on;
        end
    end

    grid on;
    xlabel(label_qError, "Interpreter", "latex");
    ylabel(label_tComp, "Interpreter", "latex");
    box on;
    switch SYSTEM_MDL
        case 0
            if iC == 2
                legend("Interpreter", "latex", "IconColumnWidth", 20, ...
                    "Location", "southwest", "Orientation", "horizontal", ...
                    "NumColumns", 2, ...
                    "BackgroundAlpha", 0.8);
            end
        case 1
            legend("Interpreter", "latex", "IconColumnWidth", 20, ...
                "Location", "northoutside", "Orientation", "vertical", ...
                "NumColumns", 3, ...
                "BackgroundAlpha", 0.8);
        case 2
            legend("Interpreter", "latex", "IconColumnWidth", 20, ...
                "Location", "southwest", "Orientation", "horizontal", ...
                "NumColumns", 2, ...
                "BackgroundAlpha", 0.8);
    end

    axis tight;

    ax = gca();
    ax.XScale = "log";
    ax.YScale = "log";
    ax.TickLabelInterpreter = "latex";

    % Axis settings
    switch SYSTEM_MDL
        case 0
            ax.XTick = [1e-4, 1e-2, 1];
            ax.XLim = [2e-5, 5];
            ax.YLim = [5e-3, 1];
        case 1
            ax.XTick = 10.^(-8:2:-1);
            ax.XLim = [5e-9, 1e-1];
            % ax.YLim = [0.5, 500];
            if iC == 1
                ax.YTick = [0.5, 1, 10, 100];
            else
                ax.YTick = [0.2, 0.5, 1, 2, 4, 8];
            end
        case 2
            ax.XTick = [1e-5, 1e-3, 1e-1];
            ax.YTick = [1e-1, 1e0, 1e1];
    end

    if SYSTEM_MDL == 2
        pdfWidth2 = 9*28.346; % width in pt;
        pdfAspectRatio2 = 1.6;
    else
        pdfWidth2 = pdfWidth;
        pdfAspectRatio2 = pdfAspectRatioDefault;% 1.4;
    end

    if SAVE_PLOTS
        saveFigureArray(fh_tComp_qError, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", false, "savePDF", true, ...
            "pdfWidth", pdfWidth2, "pdfAspectRatio", pdfAspectRatio2, ...
            "outsideLegendYShift", 0.05);
    end
end


%% Step Size over Time (ode solvers)

% Tolerance, for which to plot the time steps
tolTarget = 1e-6;

for iC = 1:nCases

    fh_hODE_time =  figure(...
        "Name", sprintf("hODE-time case %d", iC), ...
        "NumberTitle", "off", "Theme", "Light");
    ax_hODE_time = axes(fh_hODE_time);

    for iInt = 1:length(simStudyRes(iC).res)
        intID = find(pOpts.intNames == simStudyRes(iC).intDef(iInt).Name);
        if ~contains(simStudyRes(iC).intDef(iInt).Name, "VI")
            ax = ax_hODE_time;
            if pOpts.includeInt(intID)

                % Find parameter ID for target tolerance
                pID = find(simStudyRes(iC).intDef(iInt).ParamVec == tolTarget);
                hODE = diff(simStudyRes(iC).res(iInt).MBSimObj(pID).simRes.tout);
                hODE(end+1) = nan;
                % Plot
                semilogy( ...
                    simStudyRes(iC).res(iInt).MBSimObj(pID).simRes.tout, ...
                    hODE, ...
                    "DisplayName", simStudyRes(iC).intDef(iInt).Name, ...
                    "Color", pOpts.intColors(intID, :), ...
                    "LineWidth", plotLineWidth*2/3 ...
                    ...%"LineStyle", pOpts.lineStyles(intID) ...
                    );
                hold(ax_hODE_time, "on");
            end
        end
    end

    grid on;
    ylabel("$h$", "Interpreter", "latex");
    xlabel("time $t$ in s", "Interpreter", "latex");
    box on;
    switch SYSTEM_MDL
        case 0
            if ~(iFig == 2 && iC == 2) && ~(iFig == 1 && iC == 1 )
                legend("Interpreter", "latex", "IconColumnWidth", 25, ...
                    "Location", "southwest", "BackgroundAlpha", 0.85);
            end
        case 1
            if iC ~= 1
                legend("Interpreter", "latex", "IconColumnWidth", 25, ...
                    "Location", "southeast");
            end
        case 2
            legend("Interpreter", "latex", "IconColumnWidth", 25, ...
                "Location", "southeast", "BackgroundAlpha", 0.85);
    end
    axis tight;

    ax_hODE_time.XScale = "linear";
    ax_hODE_time.YScale = "log";
    ax_hODE_time.TickLabelInterpreter = "latex";

    % Axis settings
    ax_hODE_time.XLim = [0,simStudyRes(iC).res(iInt).MBSimObj(pID).simPars.tEnd];
    switch SYSTEM_MDL
        case 0
            ax_hODE_time.YLim = [1e-4, 0.04];
        case 1
            ax_hODE_time.YLim = [1e-7, 1e-2];
        case 2
            ax_hODE_time.YLim = [1e-5, 1e-1];
    end

    if SAVE_PLOTS
        saveFigureArray(fh_hODE_time, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", false, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatioDefault, ...
            "outsideLegendMatchWidth", false, ...
            "outsideLegendYShift", 0);
    end
end



%% Mean Step Size over Tolerance (ode solvers)

for iC = 1:nCases

    fh_hMean_tol =  figure(...
        "Name", sprintf("hMean-tol case %d", iC), ...
        "NumberTitle", "off", "Theme", "Light");
    ax_hMean_tol = axes(fh_hMean_tol);

    for iInt = 1:length(simStudyRes(iC).res)
        intID = find(pOpts.intNames == simStudyRes(iC).intDef(iInt).Name);
        if ~contains(simStudyRes(iC).intDef(iInt).Name, "VI")
            ax = ax_hMean_tol;
            if pOpts.includeInt(intID)

                hMean = zeros(length(simStudyRes(iC).intDef(iInt).ParamVec),1);
                for iParam = 1:length(hMean)
                    hMean(iParam) = mean(diff(simStudyRes(iC).res(iInt).MBSimObj(iParam).simRes.tout));
                end
                loglog(ax_hMean_tol, simStudyRes(iC).intDef(iInt).ParamVec, hMean, ...
                    "DisplayName", simStudyRes(iC).intDef(iInt).Name, ...
                    "Color", pOpts.intColors(intID, :), ...
                    "LineWidth", plotLineWidth, ...
                    "MarkerSize", plotMarkerSize, ...
                    "LineStyle", pOpts.lineStyles(intID), ...
                    "Marker", pOpts.lineMarkers(intID)...
                    );
                hold(ax_hMean_tol, "on");
            end
        end
    end

    grid on;
    xlabel(label_params(iFig), "Interpreter", "latex");
    ylabel(label_hMean, "Interpreter", "latex");
    box on;
    switch SYSTEM_MDL
        case 0
            if ~(iFig == 2 && iC == 2) && ~(iFig == 1 && iC == 1 )
                legend("Interpreter", "latex", "IconColumnWidth", 25, ...
                    "Location", "southeast", "BackgroundAlpha", 0.85);
            end
        case 1
            if iC ~= 1
                legend("Interpreter", "latex", "IconColumnWidth", 25, ...
                    "Location", "northwest");
            end
        case 2
            legend("Interpreter", "latex", "IconColumnWidth", 25, ...
                "Location", "northwest", "BackgroundAlpha", 0.85);
    end
    axis tight;

    ax_hMean_tol.XScale = "log";
    ax_hMean_tol.YScale = "log";
    ax_hMean_tol.TickLabelInterpreter = "latex";

    % Axis settings
    switch SYSTEM_MDL
        case 0
            %ax_hMean_tol.YTick   = [5e-3, 1e-2];
            %ax_hMean_tol.YTickLabel   = "$" + ["5\cdot10^{-3}", "10^{-2}"] + "$";
            ax_hMean_tol.YLim = [1e-3, 0.03];
        case 1
            ax_hMean_tol.XTick = flip([1e-3, 1e-5, 1e-7, 1e-9, 1e-11]);
        case 2
            ax_hMean_tol.XTick = 10.^(-11:2:-3);
            %ax_tComp_tol.YTick = [1e-5, 1e-3, 1e-1];
    end

    if SAVE_PLOTS
        saveFigureArray(fh_hMean_tol, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", false, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatioDefault, ...
            "outsideLegendMatchWidth", false, ...
            "outsideLegendYShift", 0);
    end
end

%% Compare Energy Evolutions
% for a specific relative error

fprintf("\nStarting energy comparison simulations...\n\n")

if SYSTEM_MDL ~= 0
    %return;
end

switch SYSTEM_MDL
    case 0
        % Computation time at which the simulations are (approximately) compared
        tCompTarget = [0.35, 0.25];

        % Integrators to compare
        E_compInts = ["VI-T", "ode113", "ode45", "CVODE-N"];

        % Extended simulation period
        E_tEnd = 50;
    case 1
        tCompTarget = [3, 0.6];

        E_compInts = ["VI-T", "ode15s", "ode23t", "RADAU", "CVODE-S"];
        E_tEnd = 3;
    case 2
        tCompTarget = 1.0;

        E_compInts = ["VI-T", "ode15s", "ode23t", "RADAU", "CVODE-S"];
        E_tEnd = simStudyRes(1).MBSimRef.simPars.tEnd(end);
end

res_ref_H = cell(nCases,1);
res_ref_t = cell(nCases,1);
res_comp_H = cell(nCases, length(E_compInts));
res_comp_t = cell(nCases, length(E_compInts));

for iC = 1:nCases
    % Simulate reference sim
    fprintf("\nEnergy comparison: Starting reference simulation...\n\n");
    simStudyRes(iC).MBSimRef.simPars.tEnd = E_tEnd;
    simStudyRes(iC).MBSimRef = simStudyRes(iC).MBSimRef.simulateSystem;
    simStudyRes(iC).MBSimRef = simStudyRes(iC).MBSimRef.computeEnergies;
    res_ref_H{iC} = simStudyRes(iC).MBSimRef.simRes.energies.H;
    res_ref_t{iC} = simStudyRes(iC).MBSimRef.simRes.tout;

    % Delete simulation results to free memory
    simStudyRes(iC).MBSimRef.simRes = MBSimResults;

    % Simulate comparison integrators

    for iInt = 1:length(E_compInts)
        if ismember(E_compInts(iInt), [simStudyRes(iC).intDef.Name])
            % Find integrator by name
            intIDDef = find(E_compInts(iInt) == [simStudyRes(iC).intDef.Name]);
            intID = find(pOpts.intNames == simStudyRes(iC).intDef(intIDDef).Name);

            % Find case closest to target tComp
            [~, caseID] = min(abs(simStudyRes(iC).res(intIDDef).tComp - tCompTarget(iC)));
            fprintf("\nEnergy comparison %s: Param Case %d / param %e / tComp %.4f\n\n", ...
                simStudyRes(iC).intDef(intIDDef).Name, caseID, ...
                simStudyRes(iC).intDef(intIDDef).ParamVec(caseID), ...
                simStudyRes(iC).res(intIDDef).tComp(caseID) ...
                );

            % Simulate system with longer end time
            MBSimEC = simStudyRes(iC).res(intIDDef).MBSimObj(caseID);
            MBSimEC.simPars.tEnd = E_tEnd;
            MBSimEC.solver.accurateTiming = false;
            MBSimEC = MBSimEC.simulateSystem();

            % Compute energies
            MBSimEC = MBSimEC.computeEnergies;
            res_comp_H{iC, iInt} = MBSimEC.simRes.energies.H;
            res_comp_t{iC, iInt} = MBSimEC.simRes.tout;

            clear MBSimEC;
        end
    end
end

%% Plot Energies

% Plot error w.r.t. reference instead of absolute values?
PLOT_ERROR = false;

for iC = 1:nCases

    fh_E = figure(...
        "Name", sprintf("energy case %d", iC), ...
        "NumberTitle", "off", "Theme", "Light");

    if ~PLOT_ERROR
        plot(res_ref_t{iC}, res_ref_H{iC}, ...
            "DisplayName", "Ref", ...
            "LineWidth", plotLineWidth ...
            );
    end
    hold on;
    for iInt = 1:length(E_compInts)
        if ismember(E_compInts(iInt), [simStudyRes(iC).intDef.Name])
            % Find integrator by name
            intIDDef = find(E_compInts(iInt) == [simStudyRes(iC).intDef.Name]);
            intID = find(pOpts.intNames == simStudyRes(iC).intDef(intIDDef).Name);

            if PLOT_ERROR
                % Get reference energy at simulation time grid
                res_ref_H_interp = interp1( res_ref_t{iC}, res_ref_H{iC}, ...
                    res_comp_t{iC, iInt} ...
                    );

                plot(res_comp_t{iC, iInt}, abs(res_ref_H_interp-res_comp_H{iC, iInt}), ...
                    "DisplayName", simStudyRes(iC).intDef(intIDDef).Name, ...
                    "Color", pOpts.intColors(intID, :), ...
                    "LineStyle", pOpts.lineStyles(intID), ...
                    "LineWidth", plotLineWidth ...
                    );
            else
                plot(res_comp_t{iC, iInt}, res_comp_H{iC, iInt}, ...
                    "DisplayName", simStudyRes(iC).intDef(intIDDef).Name, ...
                    "Color", pOpts.intColors(intID, :), ...
                    "LineStyle", pOpts.lineStyles(intID), ...
                    "LineWidth", plotLineWidth ...
                    );
            end
        end
    end

    grid on;
    xlim([0, E_tEnd]);
    ax = gca;
    if PLOT_ERROR
        ax.YScale = "log";
    end
    ax.TickLabelInterpreter = "latex";
    xlabel("time $t$ in s", "Interpreter", "latex");
    ylabel("energy $H$ in J", "Interpreter", "latex");
    pdfWidthE = pdfWidth;
    pdfAspectRatioE = pdfAspectRatioDefault;
    switch SYSTEM_MDL
        case 0
            if iC == 1
                legend("Interpreter", "latex", ...
                    "Location", "northwest", "IconColumnWidth", 20, ...
                    "BackgroundAlpha", 0.85);
                ax.YAxis.Exponent = -2;
            else
                ax.YAxis.Exponent = 1;
            end
        case 1
            ax.YAxis.Exponent = -2;
            if iC == 1
                ylim([0.0895, 0.0899]);
            else
                ylim([0.06, 0.091]);
            end
            if iC == 2
                legend("Interpreter", "latex", ...
                    "Location", "northeast", "IconColumnWidth", 30);
            end
        case 2
            legend("Interpreter", "latex", ...
                "Location", "northeast", "IconColumnWidth", 30);
            pdfWidthE = 10*28.346; % width in pt;
            pdfAspectRatioE = 1.4;
    end

    if SAVE_PLOTS
        saveFigureArray(fh_E, plotSaveFolder, ...
            "saveFig", true, "saveJPEG", false, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatioDefault, ...
            "outsideLegendMatchWidth", 0, ...
            "outsideLegendYShift", 0);
    end
end

%% End script
disp("Finished.");
