function [solver_cfg, sampler_cfg, ransac_cfg, opt_cfg] = parse_cfg(varargin)
    % Default configurations
    cfg = struct();

    cfg.sample_size = 14;
    cfg.solver_model = 'rat';
    cfg.solver_complexity = 2;
    cfg.final_model = 'kb';
    cfg.final_complexity = 4;

    cfg.min_trial_count = 20;
    cfg.max_trial_count = 50;
    cfg.max_num_retries = 50;
    cfg.confidence = 0.995;

    cfg.display = true;
    cfg.display_freq = 1;
    cfg.irT = 0;

    cfg.reprojT = 1.5;
    cfg.max_iter = 50;

    [cfg, ~] = cmp_argparse(cfg, varargin{:});

    % Configurations of the solver
    solver_cfg = {'sample_size', cfg.sample_size,...
                  'solver_model', cfg.solver_model,...
                  'solver_complexity', cfg.solver_complexity,...
                  'final_model', cfg.final_model,...
                  'final_complexity', cfg.final_complexity,...
    };

    % Configurations of the sampler
    sampler_cfg = {'min_trial_count', cfg.min_trial_count,...
                   'max_trial_count', cfg.max_trial_count,...
                   'max_num_retries', cfg.max_num_retries,...
                   'confidence', cfg.confidence,...
    };

    % Configurations of RANSAC
    ransac_cfg = {'display', cfg.display,...
                  'display_freq', cfg.display_freq,...
                  'irT', cfg.irT,...
    };

    % Configurations of the refinement
    opt_cfg = {'reprojT', cfg.reprojT,...
              'max_iter', cfg.max_iter,...
    };
end
