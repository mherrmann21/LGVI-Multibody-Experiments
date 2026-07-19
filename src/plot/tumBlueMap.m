function colors = tumBlueMap(n)
    %% Generate continuous color map of official TUM Blue colors
    % Returns a (n,3) matrix of colors containing n (interpolated) blue tones
    % from dark to light.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments (Input)
        % Number of colors to output
        n (1,1) double {mustBeInteger, mustBePositive} = 10;
    end
    arguments (Output)
        % (n,3) matrix of colors
        colors (:,3) double
    end
    colors = interp1([0,1], ...
        [tumColors().TUMBlue5;tumColors().TUMBlue1], ...
        linspace(0,1,n));
end
