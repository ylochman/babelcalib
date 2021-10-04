function [proj_params, errs] = fit_rat_to_rat(x0, k0, a, K, complexity)
    if nargin < 5
        complexity = 2;
    end
    options = optimoptions('lsqnonlin','Display','off',...
                            'MaxIter',100);
    X = CAM.backproject_rat(x0, [], a);
    proj_params = zeros(1,complexity);
    m = min(size(a,2),complexity);
    proj_params(1:m) = a(1:m);
    errs = errfun(proj_params, X, x0);
end

function err = errfun(proj_params, X, xd)
    Xhat = CAM.backproject_rat(xd, [], proj_params);
    Xhat = Xhat ./ vecnorm(Xhat);
    err = [X - Xhat];
    err = reshape(err(1:2,:),[],1);
end

function err = errfun2(proj_params, X, xd)
    Xhat = CAM.backproject_rat(xd, [], proj_params);
    Xhat = Xhat ./ vecnorm(Xhat);
    err = diag(X'*Xhat);
end
