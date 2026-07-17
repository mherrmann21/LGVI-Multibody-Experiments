function saveDir = sim_study_integrators_run_case(caseDefFun, runOpts)
    %% Execute one defined integrator sim study case
    arguments
        % Function handle of function that specifies the case
        caseDefFun  (1,1) function_handle

        % Global simulation study options
        runOpts     (1,1) struct
    end

    % Global options
    ACCURATE_TIMING = runOpts.accurateTiming;
    SAVE_RESULTS = runOpts.saveResults;
    resultsDir = runOpts.resultsDir;

    % Case data
    caseDef = caseDefFun();
    SYSTEM_MDL = caseDef.systemModel;
    DISSIP_CASE = caseDef.dissipationCase;
    links = caseDef.links;
    MBSim = caseDef.MBSim;
    intDef = caseDef.intDef;
    tEnd = caseDef.tEnd;
    hRef = caseDef.hRef;
    errorMarginRef = caseDef.errorMarginRef;

    % Visualize reference configuration
    MBSim.visualizeSystemRefConf;


    %% Turn on diary / write console output to file / Prepare output folder

    subFolder = sprintf("%s_simStudy_integrators__system_%d_dissip_%d", ...
        string(datetime, 'yyMMdd_HHmm'), SYSTEM_MDL, DISSIP_CASE);

    saveDir = fullfile(resultsDir, subFolder);

    % Create output folder
    if SAVE_RESULTS && ~isfolder( saveDir )
        mkdir( saveDir );
    end

    simStartTime = datetime;

    if SAVE_RESULTS
        diary(fullfile(saveDir, 'simStudy.log'));

        fprintf('Starting Log. Time: %s\n', string(simStartTime, 'dd.MM.yy, HH:mm:ss'));

        % Print host name to be able to identify PC afterwards
        % Only works for windows, see
        % https://www.mathworks.com/matlabcentral/answers/398048-how-to-get-the-name-of-the-computer-under-matlab#answer_317798
        fprintf('   Host Machine: %s\n\n', getenv('COMPUTERNAME'));

        % Print system info again for log
        fprintf("\n\n");
        printLinkProperties(links);
        printFrameProperties(MBSim.MBSys);
        printInputProperties(MBSim.MBSys);

    end

    %% Specify Simulation Parameters

    % End time
    MBSim.simPars.tEnd = tEnd;

    % Initial configuration
    q0 = ones(MBSim.MBSys.nDoF,1)*0;
    MBSim.simPars.q0 = q0;
    MBSim.simPars.qDot0 = zeros(MBSim.MBSys.nDoF,1);

    % Visualize initial config
    MBSim.visualizeSystemConfig(q0, "figureName", "visInitConf");
    title("Initial Configuration")

    %% Time step defining the comparison grid
    hComp = max(intDef(1).ParamVec);
    toutComp = (0 : hComp : hComp*floor(tEnd/hComp)).';

    %% Reference simulation

    fprintf("Starting reference simulation...\n\n");

    MBSimRef = MBSim;
    MBSimRef.Name = "Ref";

    % Solver settings
    MBSimRef.solver = MBSimIntegratorVarIntBroyden;
    MBSimRef.solver.h = hRef;
    MBSimRef.solver.JacobianIterationThreshold = 3;
    MBSimRef.solver.errorMargin = errorMarginRef;
    MBSimRef.solver.aTrapez = 1/2;

    % Start integration
    MBSimRef = MBSimRef.simulateSystem;

    % Plotting
    MBSimRef.plotAll;

    % Animate results
    MBSimRef.animateSimResults("figureName", "AnimVI");

    % Downsample reference solution to comparison time grid
    q_ref_c = interp1(MBSimRef.simRes.tout, MBSimRef.simRes.q.', ...
        toutComp, 'pchip').';


    %% Integration of comparison simulations

    nIntegrators = length(intDef);
    res = struct();

    for iInt = 1:nIntegrators
        fprintf("\nStarting integrator %d/%d: %s...\n\n", ...
            iInt, nIntegrators, intDef(iInt).Name);

        nCases = length(intDef(iInt).ParamVec);

        % Initialize result arrays for the current integrator
        res(iInt).MBSimObj = createArray(nCases, 1, "MBSimulation");
        res(iInt).qErrorNorm = nan(nCases, 1);
        res(iInt).qErrorMax  = nan(nCases, 1);
        res(iInt).qErrorMat  = nan(nCases, length(toutComp));
        res(iInt).tComp      = nan(nCases, 1);
        res(iInt).implIterMean = nan(nCases, 1);
        res(iInt).implIterMax  = nan(nCases, 1);

        for iCase = 1:nCases

            fprintf("\nIntegrator %d/%d: Starting case %d/%d...\n\n", ...
                iInt, nIntegrators, iCase, nCases);

            MBSimCase = MBSim;
            MBSimCase.Name = sprintf("%s case %d", intDef(iInt).Name, iCase);

            % Solver settings
            MBSimCase.solver = intDef(iInt).Solver;
            MBSimCase.solver.accurateTiming = ACCURATE_TIMING;

            switch intDef(iInt).Solver.type
                case "varint"
                    MBSimCase.solver.h = intDef(iInt).ParamVec(iCase);
                case "ode"
                    MBSimCase.solver.useMassMatrixForm = false;
                    AbsTol = intDef(iInt).ParamVec(iCase);
                    RelTol = AbsTol * 1e1;

                    % Check if the solver is a regular ode solver or RADAU
                    if intDef(iInt).Name == "RADAU"
                        MBSimCase.solver.solverOptions = rdpset( ...
                            'RelTol', RelTol, 'AbsTol', AbsTol);
                    else
                        MBSimCase.solver.odeObject.RelativeTolerance = RelTol;
                        MBSimCase.solver.odeObject.AbsoluteTolerance = AbsTol;
                    end
            end

            % Start integration
            MBSimCase = MBSimCase.simulateSystem;

            res(iInt).MBSimObj(iCase) = MBSimCase;

            % Check if simulation was successful
            if MBSimCase.simRes.tout(end) < 0.9*tEnd
                fprintf("Integration failed.\n")
                continue;
            end

            % Assign results
            res(iInt).tComp(iCase) = MBSimCase.simRes.metaDataSim.TotalTime;
            if intDef(iInt).Solver.type == "varint"
                res(iInt).implIterMean(iCase) = mean( ...
                    MBSimCase.simRes.solverIterations, "omitmissing");
                res(iInt).implIterMax(iCase)  = max( ...
                    MBSimCase.simRes.solverIterations, [], "omitmissing");
            end

            % Compute integration errors
            q_comp = interp1(MBSimCase.simRes.tout, MBSimCase.simRes.q.', ...
                toutComp, 'pchip').';

            res(iInt).qErrorMat(iCase,:) = vecnorm(abs(q_comp - q_ref_c));
            res(iInt).qErrorNorm(iCase)  = norm(abs(q_comp - q_ref_c))*sqrt(hComp);
            res(iInt).qErrorMax(iCase)   = max(vecnorm(abs(q_ref_c - q_comp)));
        end
    end


    %% Plotting

    fhs = gobjects(0); %#ok<*SAGROW>

    for iInt = 1:nIntegrators

        isVI = intDef(iInt).Solver.type == "varint";

        switch intDef(iInt).Solver.type
            case "varint"
                paramLabelString = "time step $h$ in s";
                paramLegendString = "$h$ = %.2e s";
            case "ode"
                paramLabelString = "AbsTol";
                paramLegendString = "AbsTol = %.0e";
        end

        fhs(end+1) = figure("Name", sprintf("%s: qErrorNorm", intDef(iInt).Name), ...
            "NumberTitle", "off");
        loglog(intDef(iInt).ParamVec, res(iInt).qErrorNorm, '-o');
        xlabel(paramLabelString, "Interpreter", "latex");
        ylabel("norm configuration error", "Interpreter", "latex");
        grid on;

        % Add convergence line to VI plot
        if isVI
            hVec = intDef(iInt).ParamVec.';
            coeff = res(iInt).qErrorNorm(4) / hVec(4)^2;
            hold on;
            loglog(hVec,  (coeff * hVec.^2), '--');
            legend(["Error", sprintf("$%.2f h^{%.1f}$", coeff, 2)], ...
                "Interpreter", "latex", "location", "best");
        end

        fhs(end+1) = figure("Name", sprintf("%s: qErrorMax", intDef(iInt).Name), ...
            "NumberTitle", "off");
        loglog(intDef(iInt).ParamVec, res(iInt).qErrorMax, '-o');
        xlabel(paramLabelString, "Interpreter", "latex");
        ylabel("max. configuration error", "Interpreter", "latex");
        grid on;

        % Add convergence line to VI plot
        if isVI
            coeff = res(iInt).qErrorMax(4) / hVec(4)^2;
            hold on;
            loglog(hVec,  (coeff * hVec.^2), '--');
            legend(["Error", sprintf("$%.2f h^{%.1f}$", coeff, 2)], ...
                "Interpreter", "latex", "location", "best");
        end

        fhs(end+1) = figure("Name", sprintf("%s: qError / time", intDef(iInt).Name), ...
            "NumberTitle", "off");
        semilogy(toutComp, res(iInt).qErrorMat, '-');
        xlabel("time $t$ in s", "Interpreter", "latex");
        ylabel("configuration error", "Interpreter", "latex");
        grid on;
        legend( ...
            arrayfun(@(x) sprintf(paramLegendString, x), intDef(iInt).ParamVec), ...
            "Interpreter", "latex");


        % Iterations over time step (VI only)
        if isVI
            fhs(end+1) = figure("Name", sprintf("%s: meanIterations", intDef(iInt).Name), ...
                "NumberTitle", "off");
            semilogx(intDef(iInt).ParamVec, res(iInt).implIterMean, '-o');
            xlabel(paramLabelString, "Interpreter", "latex");
            ylabel("mean iterations", "Interpreter", "latex");
            grid on;

            fhs(end+1) = figure("Name", sprintf("%s: maxIterations", intDef(iInt).Name), ...
                "NumberTitle", "off");
            semilogx(intDef(iInt).ParamVec, res(iInt).implIterMax, '-o');
            xlabel(paramLabelString, "Interpreter", "latex");
            ylabel("max iterations", "Interpreter", "latex");
            grid on;
        end

        % Computation time over accuracy parameter
        fhs(end+1) = figure("Name", sprintf("%s: comp. time", intDef(iInt).Name), ...
            "NumberTitle", "off");
        loglog(intDef(iInt).ParamVec, res(iInt).tComp, '-o');
        xlabel(paramLabelString, "Interpreter", "latex");
        ylabel("comp. time in s", "Interpreter", "latex");
        grid on;
    end


    %% Overall Comparison Plots

    % Mean error over computation time
    fhs(end+1) = figure("Name", "qErrorNorm / tComp", "NumberTitle", "off");
    for iInt = 1:nIntegrators
        loglog(res(iInt).tComp, res(iInt).qErrorNorm, "-o");
        hold on;
    end
    grid on;
    xlabel("comp. time in s", "Interpreter", "latex");
    ylabel("norm configuration error", "Interpreter", "latex");
    legend([intDef.Name], "Interpreter", "latex");

    % Computation time over mean error
    fhs(end+1) = figure("Name", "tComp / qErrorNorm", "NumberTitle", "off");
    for iInt = 1:nIntegrators
        loglog(res(iInt).qErrorNorm, res(iInt).tComp, "-o");
        hold on;
    end
    grid on;
    ylabel("comp. time in s", "Interpreter", "latex");
    xlabel("norm configuration error", "Interpreter", "latex");
    legend([intDef.Name], "Interpreter", "latex");


    % Save plots
    if SAVE_RESULTS
        disp("Saving figures...")
        saveFigureArray(fhs, saveDir, ...
            "saveFig", true, "saveJPEG", true, "savePDF", false);
    end


    %% Save results
    if SAVE_RESULTS

        disp('Saving Overall Results Data...')
        try
            save( ...
                fullfile(saveDir, "simStudyResults.mat"), ...
                'MBSim', 'MBSimRef', 'intDef', 'res',...
                '-mat', '-v7.3' ...
                );
        catch ME
            warning(ME.identifier, 'Could not save data:\n %s', ME.message);
        end

        fprintf('Output folder: %s\n', saveDir);
    end


    %% End script

    simStopTime = datetime;
    fprintf(...
        'Finished. Time: %s, Total duration: %s (hrs/min/s)\n', ...
        string(simStopTime, 'dd.MM.yy, HH:mm:ss'), ...
        string( duration(simStopTime-simStartTime, 'Format', 'hh:mm:ss') ) ...
        );

    % Turn off diary
    diary('off')
end
