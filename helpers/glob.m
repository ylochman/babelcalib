function [paths, dirs] = glob(dirpath, patterns)
    if isstr(patterns), patterns = {patterns};
    end

    paths = {};
    % LEVEL 1
    paths = glob_inner(dirpath, patterns);
    
    % LEVEL 2
    dirpaths = glob_inner(dirpath, '*');
    for k2=1:numel(dirpaths)
        dirpath_ = dirpaths{k2};
        if ~strcmp(dirpath_(end),'.') & isdir(dirpath_)
            paths_ = glob_inner(dirpath_, patterns);
            paths = {paths{:} paths_{:}};
        end
    end
    paths = unique(paths);

    if nargout==2
        dirs = [];
        for k1=1:numel(patterns)
            dirs = [dirs dir(fullfile(dirpath, patterns{k1}))];
        end
    end
end

function [paths, dirs] = glob_inner(dirpath, pattern)
    dirpath = GetFullPath(dirpath);
    if isstr(pattern), patterns = {pattern};
    else, patterns = pattern;
    end
    dirs = {};
    paths = {};
    for k=1:numel(patterns)
        pattern = patterns{k};
        dirs0 = dir(fullfile(dirpath, pattern));
        dirs = {dirs{:} dirs0};

        paths0 = cellfun(@(x,y) fullfile(x,y), {dirs0.folder},...
                        {dirs0.name}, 'UniformOutput', false);
        paths = {paths{:} paths0{:}};
    end
    if numel(patterns)==1
        dirs = dirs{1};
    end
end