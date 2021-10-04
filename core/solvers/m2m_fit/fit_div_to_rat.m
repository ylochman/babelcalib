function [a, errs] = fit_div_to_rat(x0, a0, lambda, K, complexity)
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
end

function err = errfun(a, X, xd)
    xdhat = CAM.project_div(X, [], a);
    err = [xd - xdhat];
    err = reshape(err(1:2,:),[],1);
end