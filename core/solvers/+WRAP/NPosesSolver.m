
classdef NPosesSolver < handle & matlab.mixin.Heterogeneous
% N Poses from N images given camera calibration

properties
    solvers = WRAP.Solver.empty;
    mss = {};
    num_imgs;
    proj_model = 'kb';
end

methods(Static)
    function this = NPosesSolver(varargin)
        this = cmp_argparse(this, varargin{:});
        extrinsic_solver = WRAP.PoseSolver();
        for k=1:this.num_imgs;
            this.solvers(k) = [extrinsic_solver];
        end
        this.mss = {this.solvers(:).mss};
    end
end

methods
    function model = fit(this, meas, idx, varinput, varargin)
        x_all = meas('pt');
        X_all = varinput('pt');
        boards = varinput('boards');
        model = varinput('model');
        backproj_fn = str2func(['RP2.backproject_' ...
                                this.proj_model]);
        if isfield(model,'Rt')
            model = rmfield(model,'Rt');
        end
        for n=1:this.num_imgs;
            x = x_all{idx(n).img}{idx(n).board};
            x = x(:,[idx(n).x{:}]);
            X = X_all{idx(n).img}{idx(n).board};
            X = X(:,[idx(n).x{:}]);
            x = backproj_fn(model.K \ x, [], model.proj_params);
            if any(isnan(x),'all') | any(abs(imag(x))>1e-10,'all')
                break
            else
                x = real(x);
            end
            m = this.solvers(n).fit(x, [], X);
            if isempty(m)
                break
            end
            model.Rt(:,:,n) = CAM.board_to_world(...
                            boards(idx(n).board).Rt, m.Rt);
        end
        if ~isfield(model,'Rt') || size(model.Rt,3)~=this.num_imgs
            model = [];
        end
    end

    function flag = is_sample_good(this, meas, idx, varinput, varargin)
        flag = true;
    end

    function flag = is_model_good(this, meas, idx, model, varargin)
        flag = this.solvers(1).is_model_good(meas, idx, model, varargin{:}) & all(all(model.Rt(:,4,2:end) < 1e6));
    end
end
end 