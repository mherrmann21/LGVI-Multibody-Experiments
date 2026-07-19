function formatAndSaveFigureFixedWidth(fh, pdfWidth, pdfAspectRatio, plotFolder, opts)
    %% Format the given figure and save to PDF with fixed width
    % The axis dimensions are computed to fit into the fixed PDF width with
    % minimal white space.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        % Handle to the figure to be saved
        fh              (1,1)

        % Desired width of the resulting pdf in pt
        pdfWidth        (1,1) double

        pdfAspectRatio  (1,1) double

        % Folder, where the figure is to be saved.
        % If empty, the figure is not saved at all.
        plotFolder      (1,1) string

        % Additional margin added to the PDF size (after the figure axes
        % have been adjusted to the size parameters) to include e.g.,
        % colorbars etc.
        % Elements: [left, right, bottom, top] cm
        opts.additionalPDFMargin (4,1) double = zeros(4,1);

        % For legends outside the axes: Distance (in cm) to shift the
        % legend up/down
        opts.outsideLegendYShift (1,1) double = 0.2;

        % For legends outside the axes: Match the legend's width to the
        % axes width?
        opts.outsideLegendMatchWidth (1,1) logical = true;

        % Font size for all axes elements
        opts.fontSize (1,1) double = 9;
    end

    %% Prepare figure

    pdfSize = [pdfWidth, pdfWidth/pdfAspectRatio]/ 28.346; % figure size in cm

    % Un-dock figure if docked
    fh.WindowStyle = "normal";

    % Position/Size

    fh.Units = 'centimeters';
    fh.Position = [20,20,pdfSize];

    drawnow;

    fh.PaperUnits = 'centimeters';
    fh.PaperPosition = [0,0,pdfSize];
    fh.PaperSize = pdfSize;


    %% Detect layout target (axes or tiledlayout)

    tl = findobj(fh, 'Type', 'tiledlayout');
    hasTL = ~isempty(tl);
    if hasTL
        % Minimum margins between the axis and the PDF/figure border
        minAxisMarginsLeftBottom = [0.02, 0.05];
        minAxisMarginsRightTop   = [0.02, 0.05];

        layoutObj = tl(1);
        layoutObj.Units = 'centimeters';
        ti = layoutObj.TightInset;

        for iCh = 1:numel(layoutObj.Children)
            layoutObj.Children(iCh).FontSize = opts.fontSize;
        end
    else
        % Minimum margins between the axis and the PDF/figure border
        minAxisMarginsLeftBottom = [0.3, 0.35];
        minAxisMarginsRightTop   = [0.15, 0.2];

        layoutObj = fh.CurrentAxes;
        layoutObj.Units = 'centimeters';
        layoutObj.FontSize = opts.fontSize;
        ti = layoutObj.TightInset;
    end

    axisMarginsLeftBottom = max([minAxisMarginsLeftBottom; ti(1:2)]);
    axisMarginsRightTop   = max([minAxisMarginsRightTop; ti(3:4)]);

    targetPos = [
        axisMarginsLeftBottom, ...
        pdfSize-axisMarginsLeftBottom-axisMarginsRightTop
        ];

    if hasTL
        layoutObj.OuterPosition = targetPos;
    else
        layoutObj.Position = targetPos;
    end

    drawnow;


    %% Expand page if figure has a legend outside the axes
    lgd = findobj(fh, 'Type', 'Legend');
    if isscalar(lgd) && ~isempty(lgd) && isvalid(lgd) && ~hasTL
        lgd.Units = 'centimeters';
        figPos = fh.Position;
        lgdPos = lgd.Position;

        % Legend bounding box in figure coordinates
        lgdLeft   = lgdPos(1);
        lgdBottom = lgdPos(2);
        lgdRight  = lgdPos(1) + lgdPos(3);
        lgdTop    = lgdPos(2) + lgdPos(4);

        extraLeft   = opts.additionalPDFMargin(1) + max(0, 1 - lgdLeft);
        extraBottom = opts.additionalPDFMargin(3) + max(0, 1 - lgdBottom);
        extraRight  = opts.additionalPDFMargin(2) + max(0, lgdRight  - figPos(3));
        extraTop    = opts.additionalPDFMargin(4) + max(0, lgdTop    - figPos(4));
    else
        extraLeft   = opts.additionalPDFMargin(1);
        extraBottom = opts.additionalPDFMargin(3);
        extraRight  = opts.additionalPDFMargin(2);
        extraTop    = opts.additionalPDFMargin(4);
    end
   
    % Adjust paper size
    if any([extraLeft, extraBottom, extraRight, extraTop])
        % Expand figure & PDF size
        newPdfSize = pdfSize + [extraLeft + extraRight, ...
            extraBottom + extraTop];

        fh.Position(3:4) = newPdfSize;
        fh.PaperSize     = newPdfSize;
        fh.PaperPosition = [0, 0, newPdfSize];
    end

    % Adjust legend
    if isscalar(lgd) && ~isempty(lgd) && isvalid(lgd) && ~hasTL
        if opts.outsideLegendMatchWidth && any([extraBottom, extraTop])
            % Set legend width to axes width and align left edges
            axPos = fh.CurrentAxes.Position;      % [x y w h]
            lgd.Position = [ ...
                axPos(1), ...          % x: align left
                lgdPos(2), ...         % y: keep MATLAB-chosen vertical placement
                axPos(3), ...          % width: match axes
                lgdPos(4) ];           % height: unchanged
        end 

        % Add shift up/down / increase/reduce gap between legend and axis
        if extraTop
            lgdPos = lgd.Position;
            lgdPos(2) = lgdPos(2) - opts.outsideLegendYShift;
            lgd.Position = lgdPos;
        end
        if extraBottom
            lgdPos = lgd.Position;
            lgdPos(2) = lgdPos(2) + opts.outsideLegendYShift;
            lgd.Position = lgdPos;
        end
    end

    %% Save figure
    if plotFolder ~= ""
        savefig(fh, fullfile(plotFolder, fh.Name));
        %exportgraphics(fh, strcat(fullfile(plotFolder, fh.Name), ".pdf"));
        print(fh, fullfile(plotFolder, fh.Name), "-dpdf", "-vector");
    end
end
