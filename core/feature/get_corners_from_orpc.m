function [corners, good_imgs, imgsize] = get_corners_from_orpc(orpc_paths, board_idxs)
    num_imgs = numel(orpc_paths);
    display(['Processing ' num2str(num_imgs) ' images.'])
    corners = [];
    good_imgs = [];
    for m=1:num_imgs
        try
            orpc = loadORPC(orpc_paths{m});
            if ~isfield(orpc, 'pts')
                corners(m).x = [];
                corners(m).cspond = [];
            else
                board_cspond = orpc.pts(:,1)'+1;
                boardidx = ismember(board_cspond, board_idxs);
                board_cspond = board_cspond(boardidx);
                corners(m).x = orpc.pts(boardidx,4:5)'+1;
                X_cspond = orpc.pts(boardidx,2)'+1;
                corners(m).cspond = [X_cspond; board_cspond];
            end
            if ~isempty(corners(m).x)
                good_imgs = [good_imgs m];
            end
        catch
            display([orpc_paths{m} ' did not return corners'])
        end
    end
    imgsize = [orpc.height, orpc.width];
end