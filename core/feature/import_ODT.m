function [corners, boards, imgsize, img_paths] = import_ODT(orpc_paths, dsc_path, tp_path, varargin)
    cfg.img_paths = [];
    cfg.board_idxs = [];
    cfg = cmp_argparse(cfg, varargin{:});

    [boards, cfg.board_idxs] = ...
                get_boards_from_dsc(dsc_path, tp_path, cfg.board_idxs);
    [corners, good_imgs, imgsize] = ...
                get_corners_from_orpc(orpc_paths, cfg.board_idxs);
    
    if ~isempty(cfg.img_paths)
        cfg.img_paths = cfg.img_paths(good_imgs);
        corners = corners(good_imgs);
    end
    img_paths = cfg.img_paths;
end
