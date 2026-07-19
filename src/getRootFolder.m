function rootPath = getRootFolder
    %% Get the absolute root folder of the repository
    rootPath = fileparts(fileparts(mfilename("fullpath")));
end