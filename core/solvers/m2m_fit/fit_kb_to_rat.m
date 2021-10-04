function [k, errs] = fit_kb_to_rat(x0, k0, a, K, complexity)
    complexity = 4;
    options = optimoptions('lsqnonlin','Display','off',...
                            'MaxIter',100);
    X = CAM.backproject_rat(x0, [], a);

    if isempty(k0)
        theta = atan2(vecnorm(X(1:2,:),2,1), X(3,:))';
        theta3 = theta.^3;
        theta5 = theta.^5;
        theta7 = theta.^7;
        theta9 = theta.^9;

        b = vecnorm(x0(1:2,:),2,1)' - theta;
        A = [theta3 theta5 theta7 theta9];
        k0 = transpose((A'*A)\(A'*b));
    end

    [k, ~, errs] = lsqnonlin(@errfun, k0, [],[], options, X, x0);
    assert(all(imag(k)<1e-7));
    k = real(k);
    errs = vecnorm(reshape(errs,2,[]));

end

function err = errfun(k, X, xd)
    xdhat = CAM.project_kb(X, [], k);
    err = [xd - xdhat];
    err = reshape(err(1:2,:),[],1);
end
