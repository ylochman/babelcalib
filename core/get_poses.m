function [model, res, corners, boards] = get_poses(intrinsics, corners, boards, imgsize, varargin)
    %%%%%%%%%%%%%%%%%%%%%%%% Configurations %%%%%%%%%%%%%%%%%%%%%%%%
    cfg.board_idxs = [];
    cfg.img_paths = [];
    cfg.save_results = [];
    cfg.refine = 1;
    cfg.debug = 0;
    [cfg, varargin] = cmp_argparse(cfg, varargin{:});
    [solver_cfg, sampler_cfg, ransac_cfg, opt_cfg] = parse_cfg(varargin{:});
    if isempty(cfg.board_idxs)
        cfg.board_idxs = 1:numel(boards);
    end
    FM = solver_cfg{8} ; % final_model

    ny = imgsize(1);
    nx = imgsize(2);

    rng(1);

    %%%%%%%%%%%%%%%%%%%%%%%% Solver %%%%%%%%%%%%%%%%%%%%%%%%
    solver = WRAP.NPosesSolver('num_imgs', numel(corners),...
                               'proj_model', FM);

    %%%%%%%%%%%%%%%%%%%%%%%% Process inputs %%%%%%%%%%%%%%%%%%%%%%%%
    % x -- homogeneous image points;
    % X -- 3D structure points; G -- dummy here
    [x, X, G] = extract_pt_from_corners_mv(corners, boards, cfg.board_idxs);

    meas = containers.Map();
    meas('corners') = corners;
    meas('pt') = x; 
    varinput = containers.Map();
    varinput('boards') = boards;
    varinput('pt') = X;
    groups = containers.Map();
    groups('pt') = G;
    groups2 = containers.Map();

    img_cspond = arrayfun(@(c) ones(1,size(c.x,2)), corners, 'UniformOutput',0);
    img_cspond = cell2mat(cellfun(@(x,y) x*y, img_cspond,...
                            num2cell(1:numel(img_cspond)), 'UniformOutput', 0));

    %%%%%%%%%%%%%%%%%%%%%%%% Normalize data %%%%%%%%%%%%%%%%%%%%%%%%
    imcenter = [nx/2+0.5; ny/2+0.5];
    A = intrinsics.K;
    intrinsics.K = eye(3);
    varinput('model') = intrinsics;
    meas_norm = normalize_meas(meas, A);
    opt_cfg{2} = opt_cfg{2} / A(1,1); % reprojT

    %%%%%%%%%%%%%%%%%%%%%%%% RANSAC + Refinement %%%%%%%%%%%%%%%%%%%%%%%%
    sampler = CalibSampler(solver.mss, groups, groups2, sampler_cfg{:});
    opt = PosesOpt(img_cspond, boards, opt_cfg{:},...
                  'proj_fn', str2func(['RP2.project_' FM]));
    display(['RANSAC-based camera pose estimation.'])
    if cfg.refine
        ransac = Ransac(solver, sampler, opt, opt, ransac_cfg{:});
    else
        ransac = Ransac(solver, sampler, opt, [], ransac_cfg{:});
    end
    [model, res, stats] = ransac.fit(meas_norm, varinput);
    if isempty(model)
        display('Did not return any model!');
        return
    end

    %%%%%%%%%%%%%%%%%%%%%%%% Unnormalize results
    opt_cfg{2} = opt_cfg{2} * A(1,1); % reprojT
    display('Initial')
    if cfg.refine
        min_res = res.info.min_res;
        min_model = res.info.min_model;
        [min_model, min_res] = unnormalize_model(FM, min_model, min_res, A, corners, boards, opt_cfg{2});
        res.info.min_res = min_res;
        res.info.min_model = min_model;

        display('Final')
    end
    [model, res] = unnormalize_model(FM, model, res, A, corners, boards, opt_cfg{2});

end

function [model, res] = unnormalize_model(proj_model, model, res, A, corners, boards, reprojT)
    model.K = A * model.K;
    res.info.residual = A(1,1) * res.info.residual;
    res.info.dx = A(1,1) * res.info.dx;
    res.errs = A(1,1) * res.errs;
    res.loss = A(1,1)^2 * res.loss;
    if nargin > 4
        [~,dx] = get_reprojerrs(corners, boards, model, proj_model);
        res.reprojerrs = vecnorm(dx);
        res.rms = rms(res.reprojerrs);
        res.sqerrs = huberErr(res.reprojerrs, reprojT);
        % res.wreprojerrs = vecnorm(res.info.w.*dx);
        % res.wrms = rms(res.wreprojerrs);
        res.wrms = rms(res.info.w.*res.reprojerrs);
        fprintf('\tfx: %3.4f  fy: %3.4f  cx: %3.4f  cy: %3.4f\n', model.K(1,1), model.K(2,2), model.K(1,3), model.K(2,3));
        fprintf('\tproj. params (%s): %3.4f %3.4f %3.4f %3.4f', proj_model, model.proj_params);
        fprintf('\n\tReproj. Err.   : %3.2f %c %3.2f (RMS %3.4f) px \n\tHuber Sqr. Err.: %3.2f %c %3.2f (RMS %3.4f) px \n\tConsensus set: %3.2f%%\n', mean(res.reprojerrs),177,std(res.reprojerrs), res.rms, mean(res.sqerrs),177,std(res.sqerrs), rms(res.sqerrs), res.ir*100);
    end
end