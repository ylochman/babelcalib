% Copyright (c) 2017 James Pritts
% 
classdef Ransac < handle
    properties 
        solver
        sampler
        eval
        lo
        display = true
        display_freq = 50;
        irT = 0;
    end
    
    methods
        function this = Ransac(solver,sampler,eval,lo,varargin)
            this = cmp_argparse(this, varargin{:});

            this.solver = solver;
            this.sampler = sampler;
            this.eval = eval;
            if nargin < 4
                this.lo = [];
            else
                this.lo = lo;
            end
        end
        
        function [loM,lo_res,lo_stats] = do_lo(this,x,M,res,varargin)
            loM = [];
            lo_res = [];
            if ~isempty(this.lo)
                    [loM,lo_res] = this.lo.fit(x,M,res,varargin{:});
            end
        end

        function  [optM,opt_res,stats] = fit(this,x,varargin)
            tic;
            stats = struct('time_elapsed', 0, ...
                           'trial_count', 1, ...
                           'N', this.sampler.max_trial_count,...
                           'sample_count', 0, ...
                           'model_count', 0, ...
                           'local_list', [], ...
                           'global_list', [],...
                           'all_losses', [], ...
                           'all_irs', [], ...
                           'global_idx_list', []);

            res = struct('loss', inf, 'ir', 0);

            optM = [];
            lo_res = res;
            opt_res = res;
            
            has_model = false;
            while true        
                
                if this.display & mod(stats.trial_count, this.display_freq)==0
                    disp(['Trial '  num2str(stats.trial_count) ' out of ' num2str(stats.N)]);
                end

                for k = 1:this.sampler.max_num_retries
                    idx = this.sampler.sample(x);
                    is_sample_good = ...
                        this.solver.is_sample_good(x,idx,varargin{:});
                    if is_sample_good
                        model_list = this.solver.fit(x,idx,varargin{:});
                        if isempty(model_list)
                            stats.all_losses = [stats.all_losses NaN];
                            stats.all_irs = [stats.all_irs NaN];
                        else
                            has_model = true;
                            break;
                        end
                    else
                        stats.all_losses = [stats.all_losses NaN];
                        stats.all_irs = [stats.all_irs NaN];
                    end
                end
                
                assert(is_sample_good, ...
                    'Could not draw a non-degenerate sample!');
                % assert(has_model,...
                    % 'Could not generate a model!');
                if ~has_model
                    optM = [];
                    opt_res = [];
                    stats = [];
                    return
                end
                
                stats.sample_count = stats.sample_count+k;
                is_model_good = false(1,numel(model_list));

                for k = 1:numel(model_list)
                    is_model_good(k) = ...
                        this.solver.is_model_good(x,idx,model_list(k),varargin{:});
                end

                model_list = model_list(is_model_good);

                if isempty(model_list)
                    stats.all_losses = [stats.all_losses NaN];
                    stats.all_irs = [stats.all_irs NaN];
                else
                    stats.model_count = stats.model_count+numel(model_list);
                    loss = inf(numel(model_list),1);
                    for k = 1:numel(model_list)
                        [loss(k),errs{k},ir{k},loss_info{k}] = ...
                            this.eval.calc_loss(x,model_list(k), ...
                                                varargin{:});

                    end
                    [~,mink] = min(loss);
                    stats.all_losses = [stats.all_losses min(loss)];
                    stats.all_irs = [stats.all_irs ir{mink}];
                    M0 = model_list(mink);
                    res0 = struct('errs', errs{mink}, ...
                                  'loss', loss(mink), ...
                                  'ir', ir{mink}, ...
                                  'info', loss_info(mink), ...
                                  'mss', {idx});

                    if (res0.ir>=this.irT) && (res0.ir >= res.ir) &&...
                       (res0.loss < res.loss)
                        M = M0;
                        res = res0;
                        stats.global_list = cat(2,stats.global_list, ...
                                struct('model',M, ...
                                        'res',res, ...
                                        'model_count', stats.model_count, ...
                                        'trial_count', stats.trial_count));
                        stats.global_idx_list = [stats.global_idx_list sum(stats.trial_count)];
                        if res.loss < opt_res.loss
                            optM = M;
                            opt_res = res;
                            opt_res.info.min_res = opt_res;
                            opt_res.info.min_model = M;
                        end
                        if ~isempty(this.lo)
                            [loM,lo_res] = this.do_lo(x,M0,res,varargin{:});
                            lo_stats = struct('model',loM, ...
                                            'res',lo_res, ...
                                            'trial_count',stats.trial_count, ...
                                            'model_count',stats.model_count);
                            if (lo_res.loss <= opt_res.loss)
                                stats.local_list = cat(2,stats.local_list,lo_stats);
                                optM = loM;
                                opt_res = lo_res;
                            end
                        end
                    
                        % Update estimate of est_trial_count, the number
                        % of trial_count to ensure we pick, with
                        % probability p, a data set with no outliers.
                        stats.N = this.sampler.update_trial_count(opt_res.info.cs);
                    end   
                end
                    
                stats.trial_count = stats.trial_count+1;

                if (stats.trial_count > stats.N)
                    break;
                end
            end

            stats.time_elapsed = toc;               
        end
    end
end