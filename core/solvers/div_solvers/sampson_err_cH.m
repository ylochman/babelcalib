function err = sampson_err_cH(params, X, x)
    % One-sided Sampson error through {c,H}-parameterization
    c = params(1:2)';
    h = params(3:end);
    h = [h(1:3) 1 h(4:5)];
    ex = sksym([c;1]);
    F = ex * [reshape(h,3,2)'; 0 0 0];
    v = diag(x' * F * X)';
    J = F(1:2,:)*X;
    err = -v ./ sum(J.^2) .* J;
    err = reshape(err(1:2,:),[],1);
end