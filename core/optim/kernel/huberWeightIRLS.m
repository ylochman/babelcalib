function weight = huberWeightIRLS(dr, reprojT)
    if size(reprojT) == 1
        reprojT = reprojT * ones(size(dr));
    end
    weight = ones(size(dr));
    cs = abs(dr) <= reprojT;
    weight(~cs) = reprojT(~cs)./abs(dr(~cs));
    weight = sqrt(weight);
end