function saveDir = sim_study_OCP_disc_run_case(caseDef, runOpts)
    %% Execute one OCP discretization study case

    arguments
        % Definition of the simulation study case
        caseDef (1,1) struct

        % Simulation study run options
        runOpts (1,1) struct
    end

    % Run options
    debugMode = runOpts.debug;
    saveResults = runOpts.saveResults;
    resultsDir = runOpts.resultsDir;
    hVec = runOpts.hVec;
    animateInitialGuess = runOpts.animateInitialGuess;

    % Case data
    systemModel = caseDef.systemModel;
    links = caseDef.links;
    MBSim = caseDef.MBSim;
    OCP = caseDef.OCP;
    computeInitialGuess = caseDef.computeInitialGuess;
    refDiscretization = caseDef.refDiscretization;


    %% Finish OCP object initialization

    OCP.MBSys = MBSystemSym(links);
    OCP.simPars = MBSim.simPars;

    OCP.nlpOpts.expand = false;

    % OCP.nlpOpts.ipopt.tol = 1e-12;
    % OCP.nlpOpts.ipopt.dual_inf_tol = 1e-10;
    % OCP.nlpOpts.ipopt.constr_viol_tol = 1e-10;
    % OCP.nlpOpts.ipopt.acceptable_tol = 1e-10;

    OCP.nlpOpts.ipopt.linear_solver = 'ma97'; % best for stiff systems!
    OCP.nlpOpts.ipopt.fixed_variable_treatment = 'relax_bounds';


    %% Visualize reference configuration and target position

    MBSim.visualizeSystemRefConf();
    coordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));


    %% Define solvers

    discNames = ["VI", "RK2-1s", "RK4-1s", "implicitMidpoint"];
    discretizations = {
        OCPIntegratorVI, OCPIntegratorRK("RK2"), ...
        OCPIntegratorRK("RK4"), OCPIntegratorImplicitMidpoint
        };

    % Make sure VI is second-order (trapezoidal rule)
    discretizations{1}.aTrapez = 0.5;

    % Time step for reference solution
    hRef  = min(hVec)/2;

    % Time step defining the comparison grid
    hComp = max(hVec);

    if debugMode
        debugDiscIndices = 1;%[1,3];
        discNames = discNames(debugDiscIndices);
        discretizations = discretizations(debugDiscIndices);
        %hVec = 2.^(-5:-1:-8);
        %hRef  = min(hVec)/2;
    end

    %% Prepare output folder and command log

    subFolder = sprintf("%s_simStudy_ocp_discretization__system_%d", ...
        string(datetime, 'yyMMdd_HHmm'), systemModel);
    saveDir = fullfile(resultsDir, subFolder);

    % Create output folder
    if saveResults && ~isfolder( saveDir )
        mkdir( saveDir );
    end

    if saveResults
        diary(fullfile(saveDir, 'simStudy.log'));
    end

    simStartTime = datetime;
    fprintf('Starting Log. Time: %s\n', string(simStartTime, 'dd.MM.yy, HH:mm:ss'));

    % Print host name to be able to identify PC afterwards
    % Only works for windows, see
    % https://www.mathworks.com/matlabcentral/answers/398048-how-to-get-the-name-of-the-computer-under-matlab#answer_317798
    fprintf('   Host Machine: %s\n\n', getenv('COMPUTERNAME'));


    %% Compute initial guess
    % with reference time step

    OCPRef = OCP;
    OCPRef.h    = hRef;
    OCPRef.Name = "Ref";

    if computeInitialGuess
        [q_init_ref, qd_init_ref, u_init_ref, MBSimIG, qOptStatic] = OCPComputeInitialGuess_InvDyn( ...
            MBSim, OCPRef, "invDynMethod", "ODE", "createDebugPlots", true);

        fh_IG_Ref = plotOCPqu( ...
            OCPRef, q_init_ref, u_init_ref, "figureName", "Initial Guess", "plotDerivatives", true);

        fh_IG_Ref(end+1) = MBSim.visualizeSystemConfig(qOptStatic, "figureName", "Ref IG Static Config");
        coordSysSE3(SE3Matrix(eye(3), OCPRef.x_TCP_F));

        % Animate results
        if animateInitialGuess
            fig = init3Dplot('Name', "Animation Initial Guess");%, "WindowStyle","normal");
            coordSysSE3(SE3Matrix(eye(3), OCPRef.x_TCP_F));
            MBSimIG.animateSimResults("figure", fig);
        end

        % Save plots
        if saveResults
            saveFigureArray(fh_IG_Ref, saveDir, "saveFig", true, "saveJPEG", true);
            close all;
        end
    else
        q_init_ref  = repmat(OCPRef.q0, [1,OCPRef.nSteps+1]);
        qd_init_ref = zeros(MBSim.MBSys.nDoF, OCPRef.nSteps+1);
        u_init_ref  = zeros(MBSim.MBSys.nInputs, OCPRef.nSteps+1);
    end

    % For contManip trajectory tracking: Update TCP final position to valid
    % final position
    if OCP.iRC(5) && systemModel == 1
        gOptStatic = MBSim.MBSys.computeFwdKin(qOptStatic);
        g_TCP = gOptStatic(:,:,MBSim.MBSys.indexTCPFrame)*MBSim.MBSys.g_B_TCP;
        x_TCP_des = g_TCP(1:3, 4);
        OCP.x_TCP_F = x_TCP_des;
        OCPRef.x_TCP_F = x_TCP_des;
    end

    %% Generate desired TCP trajectory

    if OCP.iRC(5)
        [OCPRef.x_TCP_traj, fh_traj_IG] = generateDesiredTCPTrajLinear(MBSim, OCPRef);

        if saveResults
            saveFigureArray(fh_traj_IG, saveDir, "saveFig", true, "saveJPEG", true);
            close all;
        end
    end

    %% Define and solve reference OCP

    fprintf("\nStarting reference OCP...\n\n");

    OCPRef.nlpOpts.expand = false;
    OCPRef.discretization = refDiscretization;

    OCPRef = OCPRef.initSolver("useCasadiStepFunctions", true);

    if OCPRef.useSplineInputs
        % Compute control points for initial guess
        BRef = OCPRef.getInputSplineBasisMatrix;
        u_init_ref_z =  (BRef \ u_init_ref.').';

        % Plot initial guess fit
        fh_Ref = figure("Name", "Ref: Initial Guess B-Spline Fit", "NumberTitle", "off");
        tiledlayout("vertical");
        nexttile;
        plot(OCPRef.tout, u_init_ref, "-.x", "DisplayName", "Original Data");
        hold on;
        plot(OCPRef.tout, BRef*u_init_ref_z.', "--o", "DisplayName", "Fitted Spline");
        grid on;
        colororder(lines(MBSim.MBSys.nInputs));
        legend;
        title("Spline Fit");

        nexttile;
        plot(OCPRef.tout, abs(u_init_ref.'-BRef*u_init_ref_z.'));
        grid on;
        title("Fit Error");
    else
        u_init_ref_z = u_init_ref;
        fh_Ref = gobjects(0);
    end

    % Solve Reference OCP
    if OCPRef.discretization.type == "varint"
        x_init_ref = q_init_ref;
    else
        x_init_ref = [q_init_ref; qd_init_ref];
    end

    % Plot constraint residuals of the initial guess
    fh_Ref(end+1) = OCPRef.plotConstraintResiduals(x_init_ref, u_init_ref_z, ...
        "figureName", "Constr. Res. IG");

    [x_ref, u_ref_z, sol_ref, stats_ref] = OCPRef.solve(x_init_ref, u_init_ref_z);
    if OCPRef.useSplineInputs
        u_ref = (BRef*u_ref_z.').';
    else
        u_ref = u_ref_z;
    end
    if OCPRef.discretization.type == "varint"
        q_ref = x_ref;
        %q_dot_ref = [];
    else
        q_ref = x_ref(1:OCP.MBSys.nDoF,:);
        %q_dot_ref = x_ref(OCP.MBSys.nDoF+1:end,:);
    end

    % Plot solution data
    fh_Ref(end+1) = OCPRef.plotConstraintResiduals(x_ref, u_ref_z, "figureName", "Constr. Res. Solution");
    fh_Ref(end+1:end+2) = plotOCPqu(OCPRef, q_ref, u_ref, "plotDerivatives", true);

    if OCP.iRC(5)
        fh_Ref(end+1) = plotOCPTCPTraj(MBSim, OCPRef, q_ref);
    end

    % Check if reference solution was successful
    if ~stats_ref.success
        error("Reference OCP was not solved successfully. Return message: %s.", stats_ref.return_status)
    end

    % Downsample reference solution to comparison time grid
    toutComp = (0 : hComp : hComp*round(OCP.tF/hComp)).';
    u_ref_c = interp1(OCPRef.tout, u_ref.', toutComp, 'pchip').';
    q_ref_c = interp1(OCPRef.tout, q_ref.', toutComp, 'pchip').';

    clear x_ref
    OCPRef = OCPRef.clearSolver;
    drawnow;

    % Save plots
    if saveResults
        saveFigureArray(fh_Ref, saveDir, "saveFig", true, "saveJPEG", true);
        close all;
    end

    %% Solve comparison OCPs

    nTSteps = numel(hVec);
    nInts   = numel(discNames);

    % Initialize struct array for solver metadata
    statsArr = cell(nTSteps, nInts);

    resultsArr(nTSteps, nInts) = struct( ...
        "f", [], ...
        "qErrorNorm", [], "qErrorMax", [], ...
        "uErrorNorm", [], "uErrorMax", [] ...
        );

    qErrorMat = zeros(nTSteps, nInts, length(toutComp));
    qCompMat  = zeros(nTSteps, nInts, OCP.MBSys.nDoF, length(toutComp));

    for ih = 1:nTSteps
        fprintf("\nStarting time step %d/%d: h = %f s...\n\n", ih, nTSteps, hVec(ih));

        OCPh = OCP;
        OCPh.h = hVec(ih);

        if computeInitialGuess
            [q_init_h, qd_init_h, u_init_h] = OCPComputeInitialGuess_InvDyn( ...
                MBSim, OCPh, "invDynMethod", "ODE");
        else
            q_init_h  = repmat(OCPRef.q0, [1, OCPh.nSteps+1]);
            qd_init_h = zeros(MBSim.MBSys.nDoF, OCPh.nSteps+1);
            u_init_h  = zeros(MBSim.MBSys.nInputs, OCPh.nSteps+1);
        end

        if OCPh.useSplineInputs
            % Compute control points for initial guess
            Bh = OCPh.getInputSplineBasisMatrix;
            u_init_h_z = (Bh \ u_init_h.').';

            % Plot initial guess fit
            fh_h = figure("Name", sprintf("Time Step %d: Initial Guess B-Spline Fit", ih), "NumberTitle", "off");
            tiledlayout("vertical");
            nexttile;
            plot(OCPh.tout, u_init_h, "-.x", "DisplayName", "Original Data");
            hold on;
            plot(OCPh.tout, Bh*u_init_h_z.', "--o", "DisplayName", "Fitted Spline");
            grid on;
            colororder(lines(MBSim.MBSys.nInputs));
            legend;
            title("Spline Fit");

            nexttile;
            plot(OCPh.tout, abs(u_init_h.'-Bh*u_init_h_z.'));
            grid on;
            title("Fit Error");
        else
            u_init_h_z = u_init_h;
            fh_h = gobjects(0);
        end

        % Compute desired trajectory
        if OCP.iRC(end) % Check if trajectory tracking cost is included
            OCPh.x_TCP_traj = generateDesiredTCPTrajLinear(MBSim, OCPh);
        end

        fh_h(end+1:end+2) = plotOCPqu(OCPh, q_init_h, u_init_h, ...
            "figureName", sprintf("Time Step %d Initial Guess", ih), ...
            "plotDerivatives", true);
        if saveResults
            saveFigureArray(fh_h, saveDir, "saveFig", true, "saveJPEG", true);
            close all;
        end

        for iInt = 1:nInts

            if saveResults; close all; end
            fprintf("\nTime step %d/%d: Starting discretization %d/%d (%s)...\n\n", ...
                ih, nTSteps, iInt, nInts, discNames(iInt));

            %% Define OCP

            OCPT = OCPh;
            OCPT.Name = sprintf("Time Step %d, %s", ih, discNames(iInt));
            OCPT.discretization = discretizations{iInt};

            OCPT = OCPT.initSolver;

            %% Compute Initial guess
            if OCPT.discretization.type == "varint"
                xInit = q_init_h;
            else
                xInit = [q_init_h; qd_init_h];
            end
            fh_T = OCPT.plotConstraintResiduals(xInit, u_init_h_z, "figureName", "Constr. Res. IG");

            %% Solve OCP

            [x_sol, u_sol_z, sol, stats] = OCPT.solve(xInit, u_init_h_z);

            if OCPh.useSplineInputs
                u_sol = (Bh*u_sol_z.').';
            else
                u_sol = u_sol_z;
            end

            if OCPT.discretization.type == "varint"
                q_sol = x_sol;
                q_dot_sol = [];
            else
                q_sol = x_sol(1:OCP.MBSys.nDoF,:);
                q_dot_sol = x_sol(OCP.MBSys.nDoF+1:end,:);
            end

            fh_T(2) = OCPT.plotConstraintResiduals(x_sol, u_sol_z, "figureName", "Constr. Res. Solution");
            fh_T(3:4) = plotOCPqu(OCPT, q_sol, u_sol, "q_dot", q_dot_sol, "plotDerivatives", true);

            if OCP.iRC(end) % If OCP includes trajectory tracking
                fh_T(end+1) = plotOCPTCPTraj(MBSim, OCPT, q_sol); %#ok<AGROW>
            end

            % Temp: Plot midpoint values of u
            if isa(OCPT.discretization, "OCPIntegratorImplicitMidpoint")
                figure("Name", OCPT.Name + ": midpoint u", "NumberTitle", "off");
                u_midpoint = (u_sol(:,1:end-1)+u_sol(:,2:end))/2;
                stairs(OCPT.tout, [u_midpoint,u_midpoint(:,end)].');
                grid on;
            end

            %% Store solution data

            % Metadata
            statsArr{ih, iInt} = stats;

            % Objective function value
            resultsArr(ih, iInt).f = sol.f;

            % Configuration and input error
            u_comp = interp1(OCPT.tout, u_sol.', toutComp, 'pchip').';
            q_comp = interp1(OCPT.tout, q_sol.', toutComp, 'pchip').';

            qCompMat(ih, iInt,:,:) = q_comp;
            qErrorMat(ih, iInt,:) = vecnorm(q_comp - q_ref_c);

            % Maximum errors (based on the Euclidean norm of each column of
            % inner matrix, where each column represents a time step)
            resultsArr(ih, iInt).qErrorMax  = max(vecnorm(q_ref_c - q_comp,2,1));
            resultsArr(ih, iInt).uErrorMax  = max(vecnorm(u_ref_c - u_comp,2,1));

            % Error norm / discrete L2 norm based on the Frobenius matrix norm
            resultsArr(ih, iInt).qErrorNorm = norm(abs(q_comp - q_ref_c), 'fro')*sqrt(hComp);
            resultsArr(ih, iInt).uErrorNorm = norm(abs(u_comp - u_ref_c), 'fro')*sqrt(hComp);

            if saveResults
                saveFigureArray(fh_T, saveDir, "saveFig", true, "saveJPEG", true);
                clear fh_T;
            end
            drawnow;
            clear x_sol
        end
    end

    %% Store data

    res.f = full(reshape([resultsArr.f], nTSteps, nInts));
    res.fError  = abs(res.f - full(sol_ref.f));
    res.qErrorM = full(reshape([resultsArr.qErrorMax], nTSteps, nInts));
    res.qErrorN = full(reshape([resultsArr.qErrorNorm], nTSteps, nInts));
    res.uErrorM = full(reshape([resultsArr.uErrorMax], nTSteps, nInts));
    res.uErrorN = full(reshape([resultsArr.uErrorNorm], nTSteps, nInts));

    res.iter_count   = nan(nTSteps, nInts);
    res.t_wall_total = nan(nTSteps, nInts);
    res.t_proc_total = nan(nTSteps, nInts);
    res.success      = nan(nTSteps, nInts);
    res.return_status = strings(nTSteps, nInts);
    for ih = 1:nTSteps
        for iInt = 1:nInts
            if isfield(statsArr{ih, iInt}, "iter_count")
                res.iter_count(ih, iInt) = statsArr{ih, iInt}.iter_count;
            end
            if isfield(statsArr{ih, iInt}, "t_wall_total")
                res.t_wall_total(ih, iInt) = statsArr{ih, iInt}.t_wall_total;
            end
            if isfield(statsArr{ih, iInt}, "t_proc_total")
                res.t_proc_total(ih, iInt) = statsArr{ih, iInt}.t_proc_total;
            end
            if isfield(statsArr{ih, iInt}, "success")
                res.success(ih, iInt) = statsArr{ih, iInt}.success;
            end
            if isfield(statsArr{ih, iInt}, "return_status")
                res.return_status(ih, iInt) = statsArr{ih, iInt}.return_status;
            end
        end
    end

    disp("NLP Return messages:");
    disp(res.return_status);

    %% Estimate convergence rates

    conv_q(nInts) = struct("a", [], "b", []);
    conv_u(nInts) = struct("a", [], "b", []);
    conv_f(nInts) = struct("a", [], "b", []);
    for iInt = 1:nInts
        % Error q
        iH = round(length(hVec)/2);
        conv_q(iInt).b = mean(log(res.qErrorM(2:end, iInt)./res.qErrorM(1:end-1, iInt)) ./ log(hVec(2:end)./hVec(1:end-1)));
        conv_q(iInt).a = res.qErrorM(iH, iInt) / hVec(iH)^(conv_q(iInt).b);

        % Error u
        conv_u(iInt).b = mean(log(res.uErrorM(2:end, iInt)./res.uErrorM(1:end-1, iInt)) ./ log(hVec(2:end)./hVec(1:end-1)));
        conv_u(iInt).a = res.uErrorM(iH, iInt) / hVec(iH)^(conv_u(iInt).b);

        % Error f
        conv_f(iInt).b = mean(log(res.fError(2:end, iInt)./res.fError(1:end-1, iInt)) ./ log(hVec(2:end)./hVec(1:end-1)));
        conv_f(iInt).a = res.fError(iH, iInt) / hVec(iH)^(conv_f(iInt).b);
    end

    disp("Mean Convergence Rates:")
    disp(array2table( ...
        [[conv_q.b];[conv_u.b];[conv_f.b]], ...
        "RowNames", ["Error q", "Error u", "Error f"], ...
        "VariableNames", discNames) ...
        );


    %% Plot data

    fh_eval = figure("NumberTitle", "off", "Name", "Eval qError / h");
    tiledlayout;
    nexttile;
    loglog(hVec, res.qErrorN, '-o');
    grid on;
    xlabel("time step $h$ in s", "Interpreter", "latex");
    title("configuration error (norm) over time step", "Interpreter", "latex");

    hold on;
    iDiscPlot = 1;
    loglog(hVec, conv_q(iDiscPlot).a * hVec.^conv_q(iDiscPlot).b, '--');
    legend([discNames, sprintf("$%.2f h^{%.1f}$", conv_q(iDiscPlot).a, conv_q(iDiscPlot).b)], ...
        "Interpreter", "latex", "location", "best");

    if contains(join(discNames), "RK4")
        discIndex = find(contains(discNames, "RK4"));
        hIndex = 2;
        coeff = res.qErrorN(hIndex,discIndex) / hVec(hIndex)^4;
        loglog(hVec,  (coeff * hVec.^4), "--");
        legend([discNames, ...
            sprintf("$%.2f h^{%.1f}$", conv_q(iDiscPlot).a, conv_q(iDiscPlot).b) ...
            sprintf("$%.2f h^{%.1f}$", coeff, 4) ...
            ], ...
            "Interpreter", "latex", "location", "best");
    end

    nexttile;
    loglog(hVec, res.qErrorM, '-o');
    grid on;
    xlabel("time step $h$ in s", "Interpreter", "latex");
    title("configuration error (max) over time step", "Interpreter", "latex");

    hold on;
    coeff = res.qErrorM(1) / hVec(1)^2;
    loglog(hVec,  (coeff * hVec.^2), '--');
    legend([discNames, sprintf("$%.2f h^{%.1f}$", coeff, 2)], "Interpreter", ...
        "latex", "location", "best");

    fh_eval(end+1) = figure("NumberTitle", "off", "Name", "Eval f / h");
    tiledlayout("vertical")
    nexttile;
    semilogx(hVec, res.f, '-o');
    grid on;
    legend(discNames, "Interpreter", "latex");
    xlabel("time step $h$ in s", "Interpreter", "latex");
    title("final cost function value over time step", "Interpreter", "latex");

    nexttile;
    loglog(hVec, res.fError, '-o');
    grid on;
    legend(discNames, "Interpreter", "latex", "Location", "best");
    xlabel("time step $h$ in s", "Interpreter", "latex");
    title("cost function error over time step", "Interpreter", "latex");

    hold on;
    iDiscPlot = 1;
    loglog(hVec, conv_f(iDiscPlot).a * hVec.^conv_f(iDiscPlot).b, '--');
    legend([discNames, sprintf("$%.2f h^{%.1f}$", conv_f(iDiscPlot).a, conv_f(iDiscPlot).b)], ...
        "Interpreter", "latex", "location", "best");

    if contains(join(discNames), "RK4")
        discIndex = find(contains(discNames, "RK4"));
        hIndex = 2;
        coeff = res.fError(hIndex,discIndex) / hVec(hIndex)^4;
        loglog(hVec,  (coeff * hVec.^4), "--");
        legend([discNames, ...
            sprintf("$%.2f h^{%.1f}$", conv_f(iDiscPlot).a, conv_f(iDiscPlot).b) ...
            sprintf("$%.2f h^{%.1f}$", coeff, 4) ...
            ], ...
            "Interpreter", "latex", "location", "best");
    end


    fh_eval(end+1) = figure("NumberTitle", "off", "Name", "Eval iter_count / h");
    loglog(hVec, res.iter_count, '-o');
    grid on;
    legend(discNames, "Interpreter", "latex", "location", "best");
    xlabel("time step $h$ in s", "Interpreter", "latex");
    title("iteration count over time step", "Interpreter", "latex");


    fh_eval(end+1) = figure("NumberTitle", "off", "Name", "Eval t_wall / h");
    loglog(hVec, res.t_wall_total, '-o');
    grid on;
    legend(discNames, "Interpreter", "latex", "location", "best");
    xlabel("time step $h$ in s", "Interpreter", "latex");
    title("solver time over time step", "Interpreter", "latex");



    fh_eval(end+1) = figure("NumberTitle", "off", "Name", "Eval t_wall / qError");
    loglog(res.qErrorN, res.t_wall_total, '-o');
    grid on;
    legend(discNames, "Interpreter", "latex", "location", "best");
    xlabel("configuration error", "Interpreter", "latex");
    ylabel("computation time in s", "Interpreter", "latex");
    title("configuration error over comp. time", "Interpreter", "latex");


    fh_eval(end+1) = figure("NumberTitle", "off", "Name", "Eval uError / h");
    loglog(hVec, res.uErrorM, '-o');
    grid on;
    legend(discNames, "Interpreter", "latex", "location", "best");
    xlabel("time step $h$ in s", "Interpreter", "latex");
    title("input error over time step", "Interpreter", "latex");

    hold on;
    iDiscPlot = 1;
    loglog(hVec, conv_u(iDiscPlot).a * hVec.^conv_u(iDiscPlot).b, '--');
    legend([discNames, sprintf("$%.2f h^{%.1f}$", conv_u(iDiscPlot).a, conv_u(iDiscPlot).b)], ...
        "Interpreter", "latex", "location", "best");

    if contains(join(discNames), "RK4")
        discIndex = find(contains(discNames, "RK4"));
        hIndex = 2;
        coeff = res.uErrorM(hIndex,discIndex) / hVec(hIndex)^4;
        loglog(hVec,  (coeff * hVec.^4), "--");
        legend([discNames, ...
            sprintf("$%.2f h^{%.1f}$", conv_u(iDiscPlot).a, conv_u(iDiscPlot).b) ...
            sprintf("$%.2f h^{%.1f}$", coeff, 4) ...
            ], ...
            "Interpreter", "latex", "location", "best");
    end

    fh_eval(end+1) = figure("NumberTitle", "off", "Name", "Eval t_wall / uError");
    loglog(res.uErrorM, res.t_wall_total, '-o');
    grid on;
    legend(discNames, "Interpreter", "latex", "location", "best");
    xlabel("input error", "Interpreter", "latex");
    ylabel("computation time in s", "Interpreter", "latex");
    title("input error over comp. time", "Interpreter", "latex");


    fh_eval(end+1) = figure("NumberTitle", "off", "Name", "Eval t_wall / fError");
    loglog(res.fError, res.t_wall_total, '-o');
    grid on;
    legend(discNames, "Interpreter", "latex", "location", "best");
    xlabel("cost function error", "Interpreter", "latex");
    ylabel("computation time in s", "Interpreter", "latex");
    title("cost function error over comp. time", "Interpreter", "latex");


    if saveResults
        saveFigureArray(fh_eval, saveDir, "saveFig", true, "saveJPEG", true, "savePDF", true);
    end

    %% Additional plots for configuration errors

    fh1 = gobjects(length(discNames), 1);
    fh2 = gobjects(length(discNames), 1);
    fh3 = gobjects(length(discNames), 1);
    for iInt = 1:nInts
        fh1(iInt) = figure("Name", discNames(iInt) + ": qErrorNorm / t", "NumberTitle", "off");
        semilogy(toutComp, squeeze(qErrorMat(:, iInt,:)));
        grid on;
        legend( ...
            arrayfun( @(x) sprintf("$h = 2^{%d}$ s", log2(x)), hVec), ...
            "Interpreter", "latex");
        ylabel("$|q^k - q_{\textrm{ref}}^k|$", "Interpreter", "latex");
        xlabel("time $t$ in s", "Interpreter", "latex");



        fh2(iInt) = figure("Name", discNames(iInt) + ": qComp / t", "NumberTitle", "off");
        tiledlayout("flow");

        for iDof = 1:OCP.MBSys.nDoF
            nexttile;
            plot(toutComp, squeeze(qCompMat(:, iInt, iDof, :)));
            hold on;
            plot(toutComp, q_ref_c(iDof,:));
            grid on;
            legend( ...
                [arrayfun( @(x) sprintf("$h = 2^{%d}$ s", log2(x)), hVec); "Ref"], ...
                "Interpreter", "latex");
            ylabel("$q^k$", "Interpreter", "latex");
            xlabel("time $t$ in s", "Interpreter", "latex");
        end


        fh3(iInt) = figure("Name", discNames(iInt) + ": qError / t", "NumberTitle", "off");
        tiledlayout("flow");

        for iDof = 1:OCP.MBSys.nDoF
            nexttile;
            semilogy(toutComp, abs(squeeze(qCompMat(:, iInt, iDof, :))-q_ref_c(iDof,:)));
            hold on;
            grid on;
            legend( ...
                arrayfun( @(x) sprintf("$h = 2^{%d}$ s", log2(x)), hVec), ...
                "Interpreter", "latex");
            ylabel("$q^k$", "Interpreter", "latex");
            xlabel("time $t$ in s", "Interpreter", "latex");
        end
    end


    if saveResults
        fhs = [fh1(:); fh2(:); fh3(:)];
        saveFigureArray(fhs, saveDir, "saveFig", true, "saveJPEG", true, "savePDF", false);
    end

    %% Plot reference solution

    disp('Post processing...')
    gTCPDes = SE3Matrix(eye(3), OCPRef.x_TCP_F);

    [q_dot, ~] = diff2ndOrder(q_ref, OCPRef.h);

    MBSimOCPRef = MBSim;
    MBSimOCPRef.Name = "Optimization";
    MBSimOCPRef.simRes = getSimResFromStateTrajectory(MBSim.MBSys, OCPRef.tout, q_ref, q_dot);

    MBSimOCPRef.plotAll;

    % Draw snapshots
    fig = init3Dplot('Name', "Snapshots Solution", "NumberTitle", "off");%, "WindowStyle","normal");
    coordSysSE3(gTCPDes);
    if OCPRef.nSteps < 50
        nSnapShots = OCPRef.nSteps/2+1;
    else
        nSnapShots = 20;
    end
    MBSimOCPRef.drawSnapshots("figure", fig, "nSnapShots",nSnapShots);
    TCPTraj = squeeze(MBSimOCPRef.simRes.g(1:3,4,end,:));
    plot3(TCPTraj(1,:),TCPTraj(2,:),TCPTraj(3,:), '-o');

    % Animate results
    fig = init3Dplot('Name', "Animation Solution");%, "WindowStyle","normal");
    coordSysSE3(gTCPDes);
    MBSimOCPRef.animateSimResults("figure", fig, "saveMovie", false, "fileName","example_optControl_contManip");



    %% Save overall data

    if saveResults
        disp('Saving overall results data...')
        try
            save( ...
                fullfile(saveDir, "simStudyResults.mat"), ...
                'res', 'discNames', 'hVec', 'hRef', 'OCP', 'MBSim', ...
                'OCPRef', 'MBSimOCPRef', 'q_ref', 'u_ref', ...
                'q_init_ref', 'u_init_ref', ...
                '-mat', '-v7.3' ...
                );
        catch ME
            warning(ME.identifier, 'Could not save data:\n %s', ME.message);
        end

        fprintf('Output folder: %s\n', saveDir);
    end


    %% Finish case
    simStopTime = datetime;

    fprintf(...
        'Finished. Time: %s, Total duration: %s (hrs/min/s)\n', ...
        string(simStopTime, 'dd.MM.yy, HH:mm:ss'), ...
        string( duration(simStopTime-simStartTime, 'Format', 'hh:mm:ss') ) ...
        );

    % Turn off diary
    diary('off')
end
