function [f, l, tz] = xX_to_f_l_tz(x, y, X, Y, Z, N)
    if nargin < 6 | isempty(N)
        N = size(x,2)-2;
    end
    
    c = mean(Z);
    Z = Z - c;
    sc = sqrt(mean([x.^2+y.^2; X.^2+Y.^2+Z.^2]/2,2));
    x = x ./ sc(1);
    y = y ./ sc(1);
    X = X ./ sc(2);
    Y = Y ./ sc(2);
    Z = Z ./ sc(2);

    r2 = x.^2 + y.^2;
    pows = 1:N;
    r2pows = (r2'.^pows)';

    M = [-x.*Z -y.*Z; X Y; X.*r2pows Y.*r2pows; -x -y]';
    Mm = mean(M);
    Ms = std(M);
    M0 = (M - Mm)./Ms;
    [U,S,V] = svd(M0);
    v0 = V(:,end);
    v = v0./Ms';
    params = v./v(1);
    params = params(2:end)';
    f = params(1);
    l = params(2:end-1);
    tz = params(end);
    l = (l .* (f.^2).^pows ./ f)';
    f = f .* sc(1);
    tz = sc(2) .* tz - c;
end