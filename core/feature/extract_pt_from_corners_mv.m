function [x, X, G] = extract_pt_from_corners_mv(corners, boards, board_idxs)
    % Args:
    %   x -- 2xN array -- N image points
    %   X -- 2xK array -- K planar target points
    %   X_cspond -- 1xN array -- x<->X correspondences
    
    num_imgs = numel(corners);
    x = {}; X = {}; G = {};
    for n=1:num_imgs
        for k=board_idxs
            if ~isempty(corners(n).x)
                [x{n}{k}, X{n}{k}, G{n}{k}] = ...
                    extract_pt_from_corners(...
                        corners(n).x(:,corners(n).cspond(2,:)==k),...
                        boards(k).X,...
                        corners(n).cspond(1,corners(n).cspond(2,:)==k));
            end
        end
    end
end