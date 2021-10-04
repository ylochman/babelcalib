function [proj_params, errs] = fit_fov_to_rat(x0, proj_params0, a, K, complexity)
    complexity = 1;
    X = CAM.backproject_rat(x0, [], a);
    X0 = X./vecnorm(X);

    r = vecnorm(x0(1:2,:));
    rs = linspace(0,max(r),200);
    [~,~,R,Z] = RAD.backproject_rat(rs, a);
    [proj_params, proj_params0] = fit_rad_fov(R, Z, rs);
    errs = vecnorm(reshape(errfun(proj_params,X0,x0),2,[]));
end

function err = errfun(proj_params, X, xd)
    xdhat = CAM.project_fov(X, [], proj_params);
    err = [xd - xdhat];
    err = reshape(err(1:2,:),[],1);
end

function [w, w0] = fit_rad_fov(R, Z, r)
    options = optimoptions('lsqnonlin','Display','off','MaxIter',100);
    d = sqrt(R.^2+Z.^2);
    R = R./d;
    Z = Z./d;
    w0 = 1;

    w = lsqnonlin(@errfun_rad, w0, [], [], options, R, Z, r);
    assert(all(imag(w)<1e-7));
    w = real(w);
end

function err = errfun_rad(w, R, Z, r)
    rhat = RAD.project_fov(R, Z, w);
    err = [r - rhat];
    err = reshape(err,[],1);
end