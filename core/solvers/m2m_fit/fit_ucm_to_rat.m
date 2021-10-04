function [xi, errs] = fit_ucm_to_rat(x0, xi0, a, K, complexity)
    complexity = 1;
    options = optimoptions('lsqnonlin','Display','off',...
                            'MaxIter',100);
    X = CAM.backproject_rat(x0, [], a);
    X0 = X./vecnorm(X);

    if isempty(xi0)
        xi0 = reshape((X0(1:2,:)-x0(1:2,:).*X0(3,:))./(x0(1:2,:)-X0(1:2,:)),1,[]);
        xi0 = mean(xi0);
    end
    
    [xi, ~, errs] = lsqnonlin(@errfun, xi0, 0, 2, options, X0, x0);
    assert(all(imag(xi)<1e-7));
    xi = real(xi);
    errs = vecnorm(reshape(errs,2,[]));

end

function err = errfun(xi, X, xd)
    xdhat = CAM.project_ucm(X, [], xi);
    err = [xd - xdhat];
    err = reshape(err(1:2,:),[],1);
end