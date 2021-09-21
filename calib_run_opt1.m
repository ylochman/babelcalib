init;
calib_cfg;

base = fullfile(root, 'data', 'ov_plane');
dsc_path = fullfile(base, 'ov_plane.dsc');

% Data
train.orpc = glob(fullfile(base, 'corners', 'train'), {'*.orpc'});
train.img = cellfun(@(x) [x(1:end-4) 'png'], train.orpc, 'UniformOutput',0);

test.orpc = glob(fullfile(base, 'corners', 'test'), {'*.orpc'});
test.img = cellfun(@(x) [x(1:end-4) 'png'], test.orpc, 'UniformOutput',0);

% Calibration
train_model = calibrate_OD(train.orpc, dsc_path,...
                            'img_paths', train.img, cfg{:});

% Evaluation (camera pose estimation)
test_model = get_poses_OD(test.orpc, dsc_path, 'model', train_model,...
                            'img_paths', test.img, cfg{:});