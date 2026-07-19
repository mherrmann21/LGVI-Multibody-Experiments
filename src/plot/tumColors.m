function [colorsRGB, colorsHex] = tumColors()
    %% Definitions of the official TUM Colors
    % Returns two structs with the official TUM colors:
    % * colorsRGB with RGB values (range 0-1)
    % * colorsHex with strings in Hex notation (HTML colors)
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    %% Hex Colors

    colorsHex.TUMBlue  = "#0065BD";
    colorsHex.TUMBlack = "#000000";

    % Additional blue color tones
    % 1 is the lightest (light blue)
    % 5 is the darkest (deep navy)
    colorsHex.TUMBlue1 = "#98C6EA"; % Pantone 283, Accent Color
    colorsHex.TUMBlue2 = "#64A0C8"; % Pantone 542, Accent Color
    colorsHex.TUMBlue3 = "#0073CF";
    colorsHex.TUMBlue4 = "#005293"; % Pantone 301, Secondary Color
    colorsHex.TUMBlue5 = "#003359"; % Pantone 540, Secondary Color

    % TUM accent colors
    colorsHex.TUMGreen     = "#A2AD00"; % Pantone 383, Accent Color
    colorsHex.TUMOrange    = "#E37222"; % Pantone 158, Accent Color
    colorsHex.TUMIvory     = "#DAD7CB"; % Pantone 7527, Accent Color

    % TUM diagram colors
    colorsHex.TUMDiaViolet      = "#69085A";
    colorsHex.TUMDiaDarkBlue    = "#0F1B5F";
    colorsHex.TUMDiaTurquoise   = "#00778A";
    colorsHex.TUMDiaDarkGreen   = "#007C30";
    colorsHex.TUMDiaLightGreen  = "#679A1D";
    colorsHex.TUMDiaLightYellow = "#FFDC00";
    colorsHex.TUMDiaDarkYellow  = "#F9BA00";
    colorsHex.TUMDiaDarkOrange  = "#D64C13";
    colorsHex.TUMDiaRed         = "#C4071B";
    colorsHex.TUMDiaDarkRed     = "#9C0D16";


    %% RGB colors
    fields = fieldnames(colorsHex);
    for iFld = 1:numel(fields)
        colChar = char(colorsHex.(fields{iFld}));
        colorsRGB.(fields{iFld}) = hex2dec(colChar([2:3;4:5;6:7])).' ./255;
    end
end
