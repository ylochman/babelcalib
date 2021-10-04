function [model, res, corners, boards] = calibrate(corners, boards, imgsize, varargin)
    %%%%%%%%%%%%%%%%%%%%%%%% Configure
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
    SS = solver_cfg{2} ; % sample_size
    SC = solver_cfg{4} ; % solver complexity
    TM = solver_cfg{6} ; % target_model
    TC = solver_cfg{8} ; % target_complexity

    ny = imgsize(1);
    nx = imgsize(2);

    rng(1);

    %%%%%%%%%%%%%%%%%%%%%%%% Solver
    full_solver = WRAP.rat_xX(WRAP.x7X_to_Rt_c(SS), SC);
    extrinsic_solver = WRAP.PoseSolver();
    solver = WRAP.CalibSolver('num_imgs', numel(corners),...
                              'full_solver', full_solver,...
                              'extrinsic_solver', extrinsic_solver,...
                              'target_model', TM,...
                              'target_complexity', TC);

    %%%%%%%%%%%%%%%%%%%%%%%% Process inputs
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

    img_cspond = arrayfun(@(c) ones(1,size(c.x,2)), corners, 'UniformOutput',0);
    img_cspond = cell2mat(cellfun(@(x,y) x*y, img_cspond,...
                            num2cell(1:numel(img_cspond)), 'UniformOutput', 0));

    %%%%%%%%%%%%%%%%%%%%%%%% Normalize data
    imcenter = [nx/2+0.5; ny/2+0.5];
    A = inv(CAM.make_diag_normalization(imcenter));
    meas_norm = normalize_meas(meas, A);
    opt_cfg{2} = opt_cfg{2} / A(1,1); % reprojT

    %%%%%%%%%%%%%%%%%%%%%%%% RANSAC + Refinement
    sampler = CalibSampler(solver.mss, groups, sampler_cfg{:});
    opt = CalibOpt(img_cspond, boards, opt_cfg{:},...
              'proj_fn', str2func(['RP2.project_' TM]),...
              'num_proj_params', TC);

    model_map = containers.Map({'rat','div','kb','ucm','ds','bc','eucm','fov'},{'Rational', 'Division', 'Kannala-Brandt','Unified Camera','Double Sphere','Brown-Conrady','Extended Unified Camera', 'Field of View'});

    fprintf(['RANSAC-based camera calibration.\n\tSolver projection model: Division-Even (complexity 2)\n\t Target projection model: ' model_map(TM) ' (complexity: ' num2str(TC) ')\n'])

    if cfg.refine
        ransac = Ransac(solver, sampler, opt, opt, ransac_cfg{:});
    else
        ransac = Ransac(solver, sampler, opt, [], ransac_cfg{:});
    end
    [model, res, stats] = ransac.fit(meas_norm, varinput,...
                                    'nx', nx/A(1,1), 'ny', ny/A(1,1));

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
        [min_model, min_res] = unnormalize_model(TM, min_model, min_res, A, corners, boards, opt_cfg{2});
        res.info.min_res = min_res;
        res.info.min_model = min_model;

        display('Final')
    end
    [model, res] = unnormalize_model(TM, model, res, A, corners, boards, opt_cfg{2});

    %%%%%%%%%%%%%%%%%%%%%%%% Save results
    if ~isempty(cfg.save_results)
        nowstr=num2str(yyyymmdd(datetime(floor(now),'ConvertFrom','datenum')));
        pth = [cfg.save_results, '_calib_' nowstr '.mat'];
        mkdir(fileparts(pth));
        save(pth, 'model', 'res');
        display(['Saved to ' pth])
    end
end

function [model, res] = unnormalize_model(proj_model, model, res, A, corners, boards, reprojT)
    model.proj_model = proj_model;
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
        res.wrms = rms(res.info.w.*res.reprojerrs);
        fprintf('\tfx: %3.4f  fy: %3.4f  cx: %3.4f  cy: %3.4f\n', model.K(1,1), model.K(2,2), model.K(1,3), model.K(2,3));
        fprintf('\tproj. params (%s): %3.4f %3.4f %3.4f %3.4f', proj_model, model.proj_params);
        fprintf('\n\tReproj. Err.   : %3.2f %c %3.2f (RMS %3.4f) px \n\tHuber Sqr. Err.: %3.2f %c %3.2f (RMS %3.4f) px \n\tConsensus set: %3.2f%%\n', mean(res.reprojerrs),177,std(res.reprojerrs), res.rms, mean(res.sqerrs),177,std(res.sqerrs), rms(res.sqerrs), res.ir*100);
    end
end