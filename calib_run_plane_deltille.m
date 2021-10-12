init;
calib_cfg;

base = fullfile(root, 'data', 'ov_plane');

% Data: Boards
dsc_path = fullfile(base, 'ov_plane.dsc');
tp_path = fullfile(base, 'ov_plane.tp');
board_idxs = [];

% Data: Corners
train.orpc = glob(fullfile(base, 'corners', 'train'), {'*.orpc'});
train.img = cellfun(@(x) [x(1:end-4) 'png'], train.orpc, 'UniformOutput',0);

test.orpc = glob(fullfile(base, 'corners', 'test'), {'*.orpc'});
test.img = cellfun(@(x) [x(1:end-4) 'png'], test.orpc, 'UniformOutput',0);

[train.corners, train.boards, train.imgsize] = import_ODT(...
                                    train.orpc, dsc_path, tp_path,...
                                    'img_paths', train.img,...
                                    'board_idxs', board_idxs);

[test.corners, test.boards, test.imgsize] = import_ODT(...
                                    test.orpc, dsc_path, tp_path,...
                                    'img_paths', test.img,...
                                    'board_idxs', board_idxs);

% Calibration
train_model = calibrate(train.corners, train.boards, train.imgsize, cfg{:},...
                       'img_paths', train.img, 'board_idxs', board_idxs,...
                       'save_results', fullfile(base, 'results', 'train'));

% Evaluation (camera pose estimation)
test_model = get_poses(train_model, test.corners, test.boards,...
                       test.imgsize, cfg{:},...
                       'img_paths', test.img, 'board_idxs', board_idxs,...
                       'save_results', fullfile(base, 'results', 'test'));