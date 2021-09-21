function [proj_params, errs] = fit_eucm_to_rat(x0, proj_params0, a, K, complexity)
    % Fit Extended Unified Camera Model to Rational Model

    X = CAM.backproject_rat(x0, [], a);
    X0 = X./vecnorm(X);
    if 1
        r = vecnorm(x0(1:2,:));
        rs = linspace(0,max(r),200);
        [~,~,R,Z] = RAD.backproject_rat(rs, a);
        [proj_params, proj_params0] = fit_rad_eucm(R, Z, rs);
        errs = vecnorm(reshape(errfun(proj_params,X0,x0),2,[]));
    else
        if isempty(proj_params0)
            R = vecnorm(X0(1:2,:));
            r = vecnorm(x0(1:2,:));
            b = (R./r-X0(3,:)).^2';
            A = [R.^2; -2*(R./r-X0(3,:)).*X0(3,:)]';
            b = b(r~=0,:);
            A = A(r~=0,:);
            k0 = transpose((A'*A)\(A'*b));
            alpha = k0(2);
            beta = k0(1)./(alpha^2);
            proj_params0 = [alpha beta];
            % rms(vecnorm(reshape(errfun(proj_params0,X0,x0),2,[])))
        end      
        options = optimoptions('lsqnonlin','Display','off','MaxIter',100);
        [proj_params, ~, errs] = lsqnonlin(@errfun, proj_params0, [], [], options, X0, x0);
        assert(all(imag(proj_params)<1e-7));
        proj_params = real(proj_params);
        errs = vecnorm(reshape(errs,2,[]));
    end

    % [proj_params0 proj_params]
    % display(['RMS Fitting Error: ' num2str(rms(errs)) ' px.'])
    % X2 = CAM.backproject_eucm(x0, [], proj_params);
    % x2 = CAM.project_eucm(X0, [], proj_params);
    % close all
    % fig;
    % subplot(1,2,1)
    % GRID.draw(x0)
    % GRID.draw(x2)
    % axis equal
    % subplot(1,2,2)
    % GRID.draw3d(X0)
    % GRID.draw3d(X2./vecnorm(X2))
    % axis equal
    % view(3)
    % keyboard
end

function err = errfun(proj_params, X, xd)
    xdhat = CAM.project_eucm(X, [], proj_params);
    err = [xd - xdhat];
    err = reshape(err(1:2,:),[],1);
end