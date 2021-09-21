function meas0 = normalize_meas(meas, A)

    function corners = norm_corners(corners, K)
        x_norm = arrayfun(@(c) RP2.normalize(c.x,K), corners,'UniformOutput',false);
        [corners(:).x] = deal(x_norm{:});
    end

    function c = norm_structured(c, K)
        for n=1:numel(c)
            c{n} = cellfun(@(x) RP2.normalize(x,K), c{n},'UniformOutput',0);
        end
    end

    norm_fns = containers.Map();
    norm_fns('pt') = @norm_structured;
    norm_fns('corners') = @norm_corners;
    
    meas0 = containers.Map(meas.keys, meas.values);
    for key = meas.keys
        key = key{1};
        norm_fn = norm_fns(key);
        meas0(key) = norm_fn(meas(key), A);
    end
end