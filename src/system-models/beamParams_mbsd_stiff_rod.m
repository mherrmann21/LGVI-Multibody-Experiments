function pars = beamParams_mbsd_stiff_rod()
    %% Beam parameters from Herrmann and Kotyczka (2024)
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    % Class Instance
    pars = elara.BeamParameters;
    baseParams = struct();


    %% Beam Parameters

    %%% Beam Geometry
    % with circular cross-section

    % Cross-section radius
    radius = 2e-3;

    % Cross-Section geometry
    % H/W corresponds to the diameter of the circular cross-section
    pars.height = 2*radius;
    pars.width = 2*radius;
    baseParams.A = radius^2 * pi;

    % Compute second moments of inertia (about x and y axes of the body-fixed
    % coordinate systems)
    % https://en.wikipedia.org/wiki/List_of_second_moments_of_area
    baseParams.I_x = pi/4 * radius^4;
    baseParams.I_y = pi/4 * radius^4;

    % Polar moment of inertia
    baseParams.J_P = pi/2 * radius^4;


    %%% Beam Material
    % Material: high-strength aluminum

    % Density (kg/m^3)
    baseParams.rho = 2.85e3;

    % Young's modulus (N/m^2)
    % https://de.wikipedia.org/wiki/Elastizit%C3%A4tsmodul#Typische_Zahlenwerte
    baseParams.E = 7.2E10;

    % Poisson's number
    % https://en.wikipedia.org/wiki/Poisson%27s_ratio#Poisson's_ratio_values_for_different_materials
    % PVC: https://wiki.polymerservice-merseburg.de/index.php/Poissonzahl
    baseParams.nu = 0.35;

    %%% Dissipation coefficients
    pars.d = 0;

    pars = pars.computeParameters(baseParams);
end
