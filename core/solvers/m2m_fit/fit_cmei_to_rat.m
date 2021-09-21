function [xi, errs] = fit_cmei_to_rat(x0, xi0, a, K, complexity)
    % Fit C.Mei's Unified Camera Model to Rational Model

    complexity = 1;
    options = optimoptions('lsqnonlin','Display','off',...
                            'MaxIter',100);
    X = CAM.backproject_rat(x0, [], a);
    X0 = X./vecnorm(X);

    if isempty(xi0)
        xi0 = reshape((X0(1:2,:)-x0(1:2,:).*X0(3,:))./(x0(1:2,:)-X0(1:2,:)),1,[]);
        xi0 = mean(xi0);
        % rms(vecnorm(reshape(errfun(xi0,X0,x0),2,[])))
    end
    
    [xi, ~, errs] = lsqnonlin(@errfun, xi0, 0, 2, options, X0, x0);
    assert(all(imag(xi)<1e-7));
    xi = real(xi);
    errs = vecnorm(reshape(errs,2,[]));

    % [xi0 xi]
    % display(['RMS Fitting Error: ' num2str(rms(errs)) ' px.'])
    % X2 = CAM.backproject_cmei(x0, [], xi);
    % x2 = CAM.project_cmei(X0, [], xi);
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

function err = errfun(xi, X, xd)
    xdhat = CAM.project_cmei(X, [], xi);
    err = [xd - xdhat];
    err = reshape(err(1:2,:),[],1);
end