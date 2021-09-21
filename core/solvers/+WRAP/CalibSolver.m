
classdef CalibSolver < handle & matlab.mixin.Heterogeneous
    
    properties
        full_solver = [];
        extrinsic_solver = [];
        mss = {};
        num_imgs = [];
        final_model = 'kb';
        final_complexity = 4;
        meas_types = [];
        % aspect_ratios = [1 4/3 1.5 2];
        aspect_ratios = [1:0.1:2];
    end

    methods(Static)
        function this = CalibSolver(varargin)
            this = cmp_argparse(this, varargin{:});
            assert(~isempty(this.full_solver))
            assert(~isempty(this.extrinsic_solver))
            % if isempty(this.full_solver)
            %     num_ct = this.solver_complexity+2;
            %     intrinsic_solver = str2func(['WRAP.' this.solver_model '_' num2str(num_ct) 'ct']);
            %     this.full_solver = WRAP.AutocalibUpgrade(intrinsic_solver());
            % end
            % this.extrinsic_solver = WRAP.PoseSolverCT();
            this.mss = repmat({this.extrinsic_solver.mss},1,this.num_imgs-1);
            this.mss = {this.full_solver.mss this.mss{:}};
            % this.meas_type = this.full_solver.mss.keys;
            % this.meas_type = this.meas_type{1};
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
                % gt = varargin{1};
                % vecnorm(model.proj_params - gt.proj_params)
                % vecnorm(model.K - gt.A\gt.K)
                % vecnorm(models0(1).Rt - gt.Rt(:,:,1))
                % keyboard
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
                    % close all
                    % figure;GRID.draw(backproj_fn(model.K\(corners(idx(n).img).x), [], model.proj_params),'color','k');GRID.draw(x); axis ij equal
                    % keyboard
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
                    if ~(strcmp(this.full_solver.solver_model, this.final_model) & numel(model.proj_params)==this.final_complexity)
                        fit_fn = str2func(['fit_' this.final_model '_to_' this.full_solver.solver_model]);
                        % xgrid0 = model.K \ xgrid;
                        x0 = model.K \ [corners(:).x];
                        maxrad = max(vecnorm(x0(1:2,:)));
                        xgrid0 = GRID.make(2*maxrad, 2*maxrad, 21, 'centered', true);
                        xgrid0 = xgrid0(:,vecnorm(xgrid0(1:2,:))<=maxrad+1e-7);
                        % model.proj_params
                        % maxrad
                        % try
                        model.proj_params = fit_fn(xgrid0, [],...
                                    model.proj_params, [],...
                                    this.final_complexity);
                            % model.proj_params
                            % keyboard
                        % catch
                        %     model = [];
                        % end
                    end
                    models = [models model];
                end
            end
            % size(models)
            % keyboard
        end

        function flag = is_sample_good(this, meas, idx, varinput, varargin)
            flag = true;
            % x_all = meas('ct');
            % X_all = varinput('ct');
            % for n=1:this.num_imgs
            %     X = X_all{idx(n).img}{idx(n).board};
            %     X = X(:,[idx(n).x{:}]);
            %     close all; fig; GRID.draw(X)
            %     l = cross(X(1:3,:),X(4:6,:));
            %     nonzero = ~~l(3,:);
            %     l(:,nonzero) = l(:,nonzero)./l(3,nonzero);
            %     all_lines_unique = size(uniquetol(l','ByRows',1)',2)==size(l,2);
            %     flag = flmag && all_lines_unique;
            % end
        end

        function flag = is_model_good(this, meas, idx, model, varargin)
            flag = this.full_solver.is_model_good(meas, idx, model, varargin{:}) & all(all(model.Rt(:,4,2:end) < 1e6));
        end
    end
end 