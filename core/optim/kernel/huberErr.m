function sqerr = huberErr(dr, reprojT)
    sqerr = dr.^2 / 2;
    cs = abs(dr) <= reprojT;
    sqerr(~cs) = reprojT * (abs(dr(~cs)) - reprojT/2);
end