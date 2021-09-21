function [k, errs] = fit_bc_to_rat(x0, k0, lambda, K, complexity)

    if nargin < 5
        complexity = 3;
    end
    options = optimoptions('lsqnonlin','Display','off',...
                            'MaxIter',100);
    X = CAM.backproject_rat(x0, [], lambda);
    if isempty(k0)
        rho = vecnorm(X(1:2,:)./X(3,:));
        r0 = vecnorm(x0(1:2,:));
        if complexity < 4
            pows = 2*(1:complexity);
            b = [r0 - rho]';
            A = [rho.^(pows'+1)]';
            k0 = transpose((A'*A)\(A'*b));
        else
            pows = 2*(1:complexity/2);
            b = [r0 - rho]';
            A = [rho.^(pows'+1); -r0 .* rho.^(pows')]';
            k0 = transpose((A'*A)\(A'*b));
            % vecnorm(errfun(k0, X, x0))
        end
    end

    [k, ~, errs] = lsqnonlin(@errfun, k0, [],[], options, X, x0);
    assert(all(imag(k)<1e-7));
    k = real(k);
    errs = vecnorm(reshape(errs,2,[]));

    % display(['RMS Fitting Error: ' num2str(rms(errs)) ' px.'])
    % X2 = CAM.backproject_bc(x0, [], k);
    % x2 = CAM.project_bc(X, [], k);
    % close all
    % fig;
    % subplot(1,2,1)
    % GRID.draw(x0)
    % GRID.draw(x2)
    % axis equal
    % subplot(1,2,2)
    % GRID.draw3d(X./vecnorm(X))
    % GRID.draw3d(X2./vecnorm(X2))
    % axis equal
    % view(3)
    % keyboard
end

function err = errfun(k, X, xd)
    xdhat = CAM.project_bc(X, [], k);
    err = [xd - xdhat];
    err = reshape(err(1:2,:),[],1);
end