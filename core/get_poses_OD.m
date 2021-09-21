function [model, res, corners_all, boards, imgsize, good_imgs] = get_poses_OD(orpc_paths, dsc_path, varargin)

    % Estimate camera poses from .orpc-.dsc files
    %
    % Args:
    %   orpc_paths -- N paths to .orpc files (detected corners)
    %   dsc_path -- path to .dsc file (calibration target)
    %   varargin ('param1', value1, 'param2', value2,...)
    %      board_idxs [] -- indices of boards to use
    %      img_paths [] -- list of image paths corresponding to 
    %                      orpc_paths

    cfg.board_idxs = [];
    cfg.img_paths = [];
    cfg.model = [];
    [cfg, varargin] = cmp_argparse(cfg, varargin{:});

    [boards, cfg.board_idxs] = get_boards_from_dsc(dsc_path, cfg.board_idxs);
    [corners, good_imgs, imgsize] = get_corners_from_orpc(orpc_paths,...
                                                          cfg.board_idxs);
    
    if ~isempty(cfg.img_paths)
        cfg.img_paths = cfg.img_paths(good_imgs);
        corners_all = corners;
        corners = corners_all(good_imgs);
    end

    keyboard
    [model, res] = get_poses(cfg.model, corners, boards, imgsize,...
                        'board_idxs', cfg.board_idxs,...
                        'img_paths', cfg.img_paths,...
                        varargin{:});
end