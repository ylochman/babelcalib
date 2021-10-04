function [proj_params, errs] = fit_eucm_to_rat(x0, proj_params0, a, K, complexity)
    complexity = 2;
    X = CAM.backproject_rat(x0, [], a);
    X0 = X./vecnorm(X);
    
    r = vecnorm(x0(1:2,:));
    rs = linspace(0,max(r),200);
    [~,~,R,Z] = RAD.backproject_rat(rs, a);
    [proj_params, proj_params0] = fit_rad_eucm(R, Z, rs);
    errs = vecnorm(reshape(errfun(proj_params,X0,x0),2,[]));
end

function err = errfun(proj_params, X, xd)
    xdhat = CAM.project_eucm(X, [], proj_params);
    err = [xd - xdhat];
    err = reshape(err(1:2,:),[],1);
end

function [proj_params, proj_params0] = fit_rad_eucm(R, Z, r)
    options = optimoptions('lsqnonlin','Display','off','MaxIter',100);

    b = (R./r-Z).^2';
    A = [R.^2; -2*(R./r-Z).*Z]';
    b = b(r~=0,:);
    A = A(r~=0,:);
    k0 = transpose((A'*A)\(A'*b));
    alpha = k0(2);
    beta = k0(1)./(alpha^2);
    Z0 = Z./sqrt(R.^2+Z.^2);
    xi = 1/(1-alpha)-1;
    if min(Z0)*(2*xi) + xi^2 + 1 - abs(xi^2-1) > 0
        xi = -1./min(Z0)-1e-7;
        alpha = xi./(xi+1);
        beta = nanmean(((R./r-Z).^2 + 2*alpha*(R./r-Z).*Z)./(R.^2*alpha^2));
    end
    if beta < max(-Z.^2./R.^2)
        beta = max(-Z.^2./R.^2)+1e-7;
        M = [R.^2*beta; -2*(R./r-Z).*Z; -(R./r-Z).^2]';
        M = M(r~=0,:);
        alpha = lsqnonlin(@(x) M * [x^2;x;1], alpha, [], [], options);
    end
    proj_params0 = [alpha beta];

    if any(isnan(RAD.project_eucm(R, Z, proj_params0)))
        proj_params0
        close all
        figure;
        plot(atan2(R,Z), r)
        hold on;plot(atan2(R,Z), RAD.project_eucm(R, Z, proj_params0))
        axis equal
        xlim([0 1])
        ylim([0 1])
        keyboard
    end

    proj_params = lsqnonlin(@errfun_rad, proj_params0, [-inf max(-Z.^2./R.^2)], [], options, R, Z, r);
    assert(all(imag(proj_params)<1e-7));
    proj_params = real(proj_params);
end

function err = errfun_rad(proj_params, R, Z, r)
    rhat = RAD.project_eucm(R, Z, proj_params);
    err = [r - rhat];
    err = reshape(err,[],1);
end