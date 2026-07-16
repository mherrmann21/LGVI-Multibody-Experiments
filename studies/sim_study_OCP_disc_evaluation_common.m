%% Evaluate results of a simulation study / generate publication plots
% Common plots for all systems
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
SAVE_PLOTS = true;

% Directory where all result subfolders are
%resultsDir = 'H:\Forschung\SimResults_all';
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
        %subFolder = "260313_1107_simStudy_ocp_discretization__system_3";
        subFolder = "260716_1712_simStudy_ocp_discretization__system_3";
        plotSaveSubFolder = "ocp_simstudy_planarManip";
    otherwise
        error("Not defined.");
end
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
res.uErrorM(~res.success) = nan;
res.uErrorN(~res.success) = nan;
res.iter_count(~res.success) = nan;
res.t_wall_total(~res.success) = nan;


% Specify which time quantity to use for computation time (CPU or wall)
res.tComp = res.t_wall_total;
%res.tComp = res.t_proc_total;

% Add convergence line for 4th-order RK4?
add4thOrderLine = (SYSTEM_MDL == 0 || SYSTEM_MDL == 3);

%% Plot settings

% Plot labels
text_qError = "$\epsilon_{q}$";
text_uError = "$\epsilon_{u}$";
text_fError = "$\epsilon_{{J}}$";
text_tComp = "$T_c$ in s";
text_h = "$h$ in s";

% Default for 2-plot (horizontal) layout
pdfWidth = 7.6*28.346; % width in pt

% Default value for "standard" (non-special) plots
pdfAspectRatioDefault = 1.5;


%% Generate plots

% qError / h
fh = plotQuantity( ...
    hVec, res.qErrorM, text_h, text_qError, ...
    "qError / h", discNames, true, add4thOrderLine);
switch SYSTEM_MDL
    case 0
        yticks(10.^(-8:2:-2));
        legend("Location", "southeast");
    case 1
        yticks(10.^(-7:2:-1));
        legend("Location", "northwest");
    case 3
        yticks(10.^(-7:2:-3));
        legend("Location", "southeast");
end

%% uError / h
fh(end+1) = plotQuantity( ...
    hVec, res.uErrorM, text_h, text_uError, ...
    "uError / h", discNames, true, add4thOrderLine);
switch SYSTEM_MDL
    case 0
        yticks(10.^(-8:2:-2));
        legend("Location", "southeast");
    case 1
        legend("Location", "northwest");

    case 3
        yticks(10.^(-7:2:-0));
        legend("Location", "southeast");
end

%% fError / h
fh(end+1) = plotQuantity( ...
    hVec, res.fError, text_h, text_fError, ...
    "fError / h", discNames, true, add4thOrderLine);
switch SYSTEM_MDL
    case 0
        yticks(10.^(-10:2:-4));
        legend("Location", "southeast");
    case 1
        legend("Location", "northwest");

    case 3
        yticks(10.^(-6:2:-0));
        legend("Location", "southeast");
end

% iter_count / h
fh(end+1) = plotQuantity( ...
    hVec, res.iter_count, text_h, "iterations", ...
    "iter_count / h", discNames, false, false);
ylim(round([min(res.iter_count(:)*0.9),max(res.iter_count(:)*1.1)]));

% tComp / h
fh(end+1) = plotQuantity( ...
    hVec, res.tComp, text_h, text_tComp, ...
    "tComp / h", discNames, false, false);
legend("Visible", "off");

if SYSTEM_MDL == 1
    ylim([5,2000]);
    yticks([10,100,1000]);
end

%% tComp / qError
fh(end+1) = plotQuantity( ...
    res.qErrorM, res.tComp, text_qError, text_tComp, ...
    "tComp / qError", discNames, false, false);
switch SYSTEM_MDL
    case 0
        xticks(10.^(-6:1:-2));
        legend("Location", "southwest");
    case 1
        xticks(10.^(-7:2:-1));
        legend("Location", "southeast");
        ylim([5,2000]);
        yticks([10,100,1000]);
    case 3
        xticks(10.^(-7:2:-3));
        legend("Location", "southwest");
end
legend("Visible", "off");


%% tComp / uError
fh(end+1) = plotQuantity( ...
    res.uErrorM, res.tComp, text_uError, text_tComp, ...
    "tComp / uError", discNames, false, false);
switch SYSTEM_MDL
    case 0
        xticks(10.^(-6:1:-2));
        legend("Location", "southwest");
    case 1
        legend("Location", "southeast");
        ylim([5,2000]);
        yticks([10,100,1000]);
    case 3
        xticks(10.^(-7:2:-0));
        legend("Location", "southwest");
end
legend("Visible", "off");

% tComp / fError
fh(end+1) = plotQuantity( ...
    res.fError, res.tComp, text_fError, text_tComp, ...
    "tComp / fError", discNames, false, false);
switch SYSTEM_MDL
    case 0
        xticks(10.^(-10:2:-4));
        legend("Location", "northwest");
    case 1
        legend("Location", "southeast");
        ylim([5,2000]);
        yticks([10,100,1000]);
    case 3
        xticks(10.^(-6:2:-0));
        legend("Location", "northwest");
end
legend("Visible", "off");

if SAVE_PLOTS
    plotSaveFolder = fullfile(plotSaveDir, plotSaveSubFolder);
    if ~isfolder( plotSaveFolder )
        mkdir( plotSaveFolder );
    end
    saveFigureArray(fh, plotSaveFolder, ...
        "saveFig", true, "saveJPEG", true, "savePDF", true, ...
        "pdfWidth", pdfWidth, ...
        "pdfAspectRatio", pdfAspectRatioDefault);
end


%% Local functions

function fh = plotQuantity(xVals, yVals, xLabelText, yLabelText, figName, dataNames, add2OConvLine, add4OConvLine)
    arguments
        xVals           (:,:) double
        yVals           (:,:) double
        xLabelText      (1,1) string
        yLabelText      (1,1) string
        figName         (1,1) string
        dataNames       (:,1) string
        add2OConvLine   (1,1) logical
        add4OConvLine   (1,1) logical
    end
    lineWidth = 1.5;
    lineWidthConv = 0.8;
    markerSize = 4.5;
    %lineColors = lines(size(yVals, 2));
    lineColors = [
        tumColors().TUMBlue;
        tumColors().TUMOrange;
        tumColors().TUMGreen;
        tumColors().TUMDiaViolet;
        ];
    lineStyles = ["-", "-", "-", "-"];
    lineMarkers = ["o", "square", "+", "x"];

    fh = figure("NumberTitle", "off", "Name", figName, "Theme", "Light");

    % 2nd-order convergence line
    if add2OConvLine
        ihCoeff = 2;
        coeff2O = yVals(ihCoeff,1) / xVals(ihCoeff)^2 * 1.3;
        loglog(xVals,  (coeff2O * xVals.^2), "k--", ...
            "LineWidth", lineWidthConv, ...
            "DisplayName", sprintf("$%.1f h^{%.0f}$", coeff2O, 2));
        hold on;
    end

    % 4th-order convergence line
    if add4OConvLine
        ihCoeff = 2;
        coeff4O = yVals(ihCoeff,3) / xVals(ihCoeff)^4 * 1.2;
        loglog(xVals,  (coeff4O * xVals.^4), "k-.", ...
            "LineWidth", lineWidthConv, ...
            "DisplayName", sprintf("$%.1f h^{%.0f}$", coeff4O, 4));
        hold on;
    end

    for iPlot = 1:size(yVals, 2)
        if size(xVals, 2) > 1
            xValsPlot = xVals(:,iPlot);
        else
            xValsPlot = xVals;
        end
        loglog(xValsPlot, yVals(:,iPlot), ...
            "LineWidth", lineWidth, ...
            "Color", lineColors(iPlot,:), ...
            "LineStyle", lineStyles(iPlot), ...
            "Marker", lineMarkers(iPlot), ...
            "MarkerSize", markerSize, ...
            "DisplayName", dataNames(iPlot) ...
            );
        hold on;
    end
    grid on;
    xlabel(xLabelText, "Interpreter", "latex");
    ylabel(yLabelText, "Interpreter", "latex");
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    legend("Interpreter", "latex", "location", "best", ...
        "BackgroundAlpha", 0.85);
end