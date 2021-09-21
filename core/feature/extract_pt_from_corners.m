function [x, X, groups] = extract_pt_from_corners(x, X, X_cspond, varargin)
    % Args:
    %   x -- 2xN array -- N image points
    %   X -- 2xK array -- K planar target points
    %   X_cspond -- 1xN array -- x<->X correspondences
    
    x = [x;ones(1,size(x,2))];
    X = [X(:,X_cspond);zeros(1,size(X_cspond,2))];
    groups = ones(1,size(x,2));
end