function hf = fig(varargin)
    hf = figure('units','normalized', 'outerposition', [0 0 1 1], varargin{:});
    hf = colordef(hf, 'white'); hf.Color = 'w';
    set(gca,'Units','normalized','Position',[0.2 0.2 0.7 0.7])
    axis xy equal
end