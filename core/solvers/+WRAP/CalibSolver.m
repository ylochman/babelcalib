
classdef CalibSolver < handle & matlab.mixin.Heterogeneous
    
    properties
        full_solver = [];
        extrinsic_solver = [];
        mss = {};
        num_imgs = [];
        target_model = 'kb';
        target_complexity = 4;
        meas_types = [];
        aspect_ratios = [1:0.1:2];
    end

    methods(Static)
        function this = CalibSolver(varargin)
            this = cmp_argparse(this, varargin{:});
            assert(~isempty(this.full_solver))
            assert(~isempty(this.extrinsic_solver))
            this.mss = repmat({this.extrinsic_solver.mss},1,this.num_imgs-1);
            this.mss = {this.full_solver.mss this.mss{:}};
            this.meas_types = arrayfun(@(n) first_el(this.mss{n}.keys),...
                                        1:this.num_imgs, 'UniformOutput', 0);
        end
    end

    methods
        function models = fit(this, meas, idx, varinput, varargin)
            cfg.nx = 1;
            cfg.ny = 1;
            [cfg, varargin] = cmp_argparse(cfg, varargin);
            corners = meas('corners');
            boards = varinput('boards');

            % Solve for projection + extrinsic
            n = 1;
            x = meas(this.meas_types{n});
            x = x{idx(n).img}{idx(n).board};
            X = varinput(this.meas_types{n});
            X = X{idx(n).img}{idx(n).board};
            models0 = [];
            for a=this.aspect_ratios
                model0 = this.full_solver.fit(x./[1;a;1], idx(n).x, X, corners(idx(n).img).x, varargin{:});
                if ~isempty(model0)
                    new_K = arrayfun(@(m) m.K.*[1 1 1; 1 a a; 1 1 1], model0, 'UniformOutput', 0);
                    [model0(:).K] = deal(new_K{:});
                    models0 = [models0 model0];
                end
            end
            % Solve for the rest (n-1) extrinsics
            models = [];
            backproj_fn = str2func(['RP2.backproject_' ...
                                        this.full_solver.solver_model]);
            for k1=1:numel(models0)
                model = models0(k1);
                model.Rt = CAM.board_to_world(...
                                boards(idx(1).board).Rt, model.Rt);
                for n=2:this.num_imgs;
                    x = meas(this.meas_types{n});
                    x = x{idx(n).img}{idx(n).board};
                    x = x(:,[idx(n).x{:}]);
                    X = varinput(this.meas_types{n});
                    X = X{idx(n).img}{idx(n).board};
                    X = X(:,[idx(n).x{:}]);
                    x = reshape(backproj_fn(model.K \ reshape(x,3,[]), [], model.proj_params),size(x,1),[]);
                    if any(isnan(x))
                        break
                    end
                    m = this.extrinsic_solver.fit(x, [], X);
                    if isempty(m)
                        break
                    end
                    model.Rt(:,:,n) = CAM.board_to_world(...
                                    boards(idx(n).board).Rt, m.Rt);
                end
                if size(model.Rt,3)==this.num_imgs
                    if ~(strcmp(this.full_solver.solver_model, this.target_model) & numel(model.proj_params)==this.target_complexity)
                        fit_fn = str2func(['fit_' this.target_model '_to_' this.full_solver.solver_model]);
                        x0 = model.K \ [corners(:).x];
                        maxrad = max(vecnorm(x0(1:2,:)));
                        xgrid0 = GRID.make(2*maxrad, 2*maxrad, 21, 'centered', true);
                        xgrid0 = xgrid0(:,vecnorm(xgrid0(1:2,:))<=maxrad+1e-7);
                        
                        model.proj_params = fit_fn(xgrid0, [],...
                                    model.proj_params, [],...
                                    this.target_complexity);
                    end
                    models = [models model];
                end
            end
        end

        function flag = is_sample_good(this, meas, idx, varinput, varargin)
            flag = true;
        end

        function flag = is_model_good(this, meas, idx, model, varargin)
            flag = this.full_solver.is_model_good(meas, idx, model, varargin{:}) & all(all(model.Rt(:,4,2:end) < 1e6));
        end
    end
end 