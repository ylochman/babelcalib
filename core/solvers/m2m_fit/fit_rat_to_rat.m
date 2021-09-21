function [proj_params, errs] = fit_rat_to_rat(x0, k0, a, K, complexity)

    complexity = 2;
    options = optimoptions('lsqnonlin','Display','off',...
                            'MaxIter',100);
    X = CAM.backproject_rat(x0, [], a);
    proj_params = a(1:2);
    errs = errfun(proj_params, X, x0);
    % if isempty(k0)
    %     % Algebraic fit for initialization
    %     r2 = x0(1,:).^2 + x0(2,:).^2;
    %     r = sqrt(r2);
    %     r4 = r2.^2;
    %     R = sqrt(X(1,:).^2+X(2,:).^2);
    %     b = [X(3,:).*r-R]';
    %     A = [R.*r2; R.*r4]';
    %     proj_params0 = transpose((A'*A)\(A'*b));
    % end
    % X = X ./ vecnorm(X);
    % [proj_params, ~, errs] =...
    %         lsqnonlin(@errfun2, proj_params0, [],[], options, X, x0);
    % assert(all(imag(proj_params)<1e-7));
    % k = real(proj_params);
    % errs = vecnorm(reshape(errs,2,[]));
    % display(['RMS Fitting Error: ' num2str(rms(errs)) ' mm.'])
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
