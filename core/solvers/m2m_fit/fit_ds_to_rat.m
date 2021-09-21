function [k, errs] = fit_ds_to_rat(x0, k0, a, K, complexity)
    % Fit Double Sphere Camera Model to Rational Model

    complexity = 2;
    options = optimoptions('lsqnonlin','Display','off',...
                            'MaxIter',100);
    X = CAM.backproject_rat(x0, [], a);
    X0 = X./vecnorm(X);

    if isempty(k0)
        R = vecnorm(X0(1:2,:));
        Z = X0(3,:);
        A = R./Z - Z;
        B = R.^2 + Z.^2;
        C = R./Z;
        data = [A;B;C;Z];
        k0 = solver_ds_to_rat(reshape(data(:,1:2),[],1));
        k0 = k0(:,all(abs(imag(k0))<1e-7));
        k0(1,:) = k0(1,:)./(k0(1,:)+1);
        errs = [];
        for kk=1:size(k0,2)
            errs = [errs rms(vecnorm(reshape(errfun(k0(:,kk)',X0,x0),2,[])))];
        end
        [~,kk]=min(errs);
        k0 = k0(:,kk)';
        % gamma0 = reshape((X0(1:2,:)-x0(1:2,:).*X0(3,:))./(x0(1:2,:)-X0(1:2,:)),1,[]);
        % gamma0 = nanmean(gamma0);
        % alpha0 = gamma0/(gamma0+1);
        % xi0 = 0;
        % k0 = [xi0 alpha0];
        % % k0
        % % rms(vecnorm(reshape(errfun(k0,X0,x0),2,[])))
        % % keyboard
    end

    [k, ~, errs] = lsqnonlin(@errfun, k0, [], [], options, X0, x0);
    assert(all(imag(k)<1e-7));
    k = real(k);
    errs = vecnorm(reshape(errs,2,[]));

    % % [k0 k]
    % display(['RMS Fitting Error: ' num2str(rms(errs)) ' mm.'])
    % keyboard
    % X2 = CAM.backproject_ds(x0, [], k);
    % x2 = CAM.project_ds(X0, [], k);
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

function err = errfun(k, X, xd)
    xdhat = CAM.project_ds(X, [], k);
    err = [xd - xdhat];
    err = reshape(err(1:2,:),[],1);
end