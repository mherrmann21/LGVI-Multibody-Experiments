function rootPath = getRootFolder
    %% Get the absolute root folder of the repository

    % Absolute file path of this function
    funPath = mfilename("fullpath");
    filepath = fileparts(funPath);

    % Get individual folders
    components = split(filepath, filesep);

    % Get root path
    rootPath = fullfile(components{1:end-1});
end