function saveFigureArray(figs, saveDir, opts)
    %% Save all figures in a figure array to file
    % The name of the file is taken from the figure name;
    % optionally, a prefix can be added before the struct field name.
    %
    % ToDo: Add options to the saved figures, if needed
    %
    % Note: For plots with large amounts of data (lots of data points),
    % the generated files might be excessively large with default settings.
    % Hence, make sure that the default .mat file format in the MATLAB
    % preferences is 7.3. (Preferences / General / Mat Files)
    % https://www.mathworks.com/matlabcentral/answers/1575343-error-using-savefig-and-saveas
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        % Struct containing the figure handles as fields
        figs (:,1)

        % String with full path to the folder, where the files should be saved
        saveDir    (1,1) string

        % Optional prefix for the file name, which is added before the
        % struct field name
        opts.namePrefix (1,1) string = "";

        opts.saveJPEG (1,1) logical = true % Save figure as jpeg?
        opts.saveFig  (1,1) logical = true % Save figure as fig?
        opts.savePDF  (1,1) logical = false; % Save as formatted pdf?

        % PDF formatting sizes
        opts.pdfWidth       (1,1) double = 10*28.346; % Page width in pt
        opts.pdfAspectRatio (1,1) double = 3/2;

        % Additional margin added to the PDF size (after the figure axis
        % have been adjusted to the size parameters) to include e.g.,
        % colorbars etc.
        % Elements: [left, right, bottom, top]
        opts.additionalPDFMargin (4,1) double = zeros(4,1);

        % For legends outside the axes: Distance (in cm) to shift the
        % legend up/down
        opts.outsideLegendYShift (1,1) double = 0.2;

        % For legends outside the axes: Match the legend's width to the
        % axis width?
        opts.outsideLegendMatchWidth (1,1) logical = true;

        % Font size for all axis elements
        opts.fontSize (1,1) double = 9;
    end

    for iFig = 1:numel(figs)

        fileName = strcat(opts.namePrefix, figs(iFig).Name);

        % Remove invalid characters from file name (tested for windows)
        fileName = erase(fileName, ["<", ">", "/", ":", "*", """", "|", "?"]);

        % Remove double spaces
        fileName = replace(fileName, "  ", " ");

        % Get full file path
        filePath = fullfile( saveDir, fileName);

        % Little bit of formatting
        figs(iFig).Theme = "light";

        try
            % Save figure
            if opts.saveFig
                savefig(figs(iFig), filePath + ".fig");
            end
            if opts.saveJPEG
                exportgraphics(figs(iFig), filePath + ".jpeg");
            end
            if opts.savePDF
                % Change figure name to file name, which is used in the pdf save function
                figs(iFig).Name = fileName;

                formatAndSaveFigureFixedWidth(figs(iFig), ...
                    opts.pdfWidth, opts.pdfAspectRatio, saveDir, ...
                    "additionalPDFMargin", opts.additionalPDFMargin, ...
                    "outsideLegendYShift", opts.outsideLegendYShift, ...
                    "outsideLegendMatchWidth", opts.outsideLegendMatchWidth, ...
                    "fontSize", opts.fontSize...
                    );
            end
        catch ME
            warning( ...
                ME.identifier, ...
                'Could not save figure:\n %s', ...
                ME.message ...
                );
        end
    end
end
