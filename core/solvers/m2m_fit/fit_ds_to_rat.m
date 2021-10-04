function [k, errs] = fit_ds_to_rat(x0, k0, a, K, complexity)
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
    end

    [k, ~, errs] = lsqnonlin(@errfun, k0, [], [], options, X0, x0);
    assert(all(imag(k)<1e-7));
    k = real(k);
    errs = vecnorm(reshape(errs,2,[]));
end

function err = errfun(k, X, xd)
    xdhat = CAM.project_ds(X, [], k);
    err = [xd - xdhat];
    err = reshape(err(1:2,:),[],1);
end