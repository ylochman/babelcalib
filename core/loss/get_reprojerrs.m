function [errs, dx] = get_reprojerrs(corners, boards, model, proj_model)
    if nargin < 4
        proj_model = 'sc';
    end
    distortion_fn = str2func(['RP2.project_' proj_model]);

    [X, Xcspond] = GRID.board_to_world(boards);
    x = [];
    xhat = [];
    for n=1:numel(corners)
        x0 = corners(n).x;
        X0 = arrayfun(@(c1,c2) X(:,Xcspond(1,:)==c1 & Xcspond(2,:)==c2), corners(n).cspond(1,:),corners(n).cspond(2,:),'UniformOutput',0);
        X0 = [X0{:}];
        xhat0 = RP2.renormI(model.K * distortion_fn(model.Rt(:,:,n) * RP3.homogenize(X0), [], model.proj_params));
        x = [x x0];
        xhat = [xhat xhat0];
    end
    dx = xhat(1:2,:) - x(1:2,:);
    errs = vecnorm(dx);
end