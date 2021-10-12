init;
calib_cfg;

base = fullfile(root, 'data', 'ov_plane');

% Data
train = load(fullfile(base, 'train'));
test = load(fullfile(base, 'test'));

% Calibration
train_model = calibrate(train.corners, train.boards, train.imgsize, cfg{:},...
                        'save_results', fullfile(base, 'results', 'train'));

% Evaluation (camera pose estimation)
test_model = get_poses(train_model, test.corners, test.boards,...
                       test.imgsize, cfg{:},...
                       'save_results', fullfile(base, 'results', 'test'));