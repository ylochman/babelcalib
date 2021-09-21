function [a, errs] = fit_sc_to_rat(x0, a0, lambda, K, complexity)
    % Fit Scaramuzza to Rational

    if nargin < 5
        complexity = 3;
    end
    options = optimoptions('lsqnonlin','Display','off',...
                            'MaxIter',100);
    X = CAM.backproject_rat(x0, [], lambda);
    if isempty(a0)
        r0 = vecnorm(x0(1:2,:));
        b = [x0(1,:).*X(3,:)-X(1,:) x0(2,:).*X(3,:)-X(2,:)]';
        A = [X(1,:).*r0.^([2:complexity+1]') ...
             X(2,:).*r0.^([2:complexity+1]')]';
        a0 = transpose((A'*A)\(A'*b));
    end
    a = a0;

    % [a, ~, errs] = lsqnonlin(@errfun, a0, [],[], options, X, x0);
    % assert(all(imag(a)<1e-7));
    % a = real(a);
    % errs = vecnorm(reshape(errs,2,[]));

    % max(abs(errfun(a0, X, x0)))
    
    % display(['RMS Fitting Error: ' num2str(rms(errs)) ' px.'])
    % X2 = CAM.backproject_cmei(x0, [], xi);
    % x2 = CAM.project_cmei(X, [], xi);
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

function err = errfun(a, X, xd)
    xdhat = CAM.project_sc(X, [], a);
    err = [xd - xdhat];
    err = reshape(err(1:2,:),[],1);
end