classdef rat_xX < WRAP.RationalSolver & dynamicprops
    % Estimated parameters: lambda1, lambda2, ...

    properties
        Rtc_solver = [];
        complexity = 3;
        N = [];
    end
    
    methods
        function this = rat_xX(solver, complexity)
            this = this@WRAP.RationalSolver(solver.mss);
            prop = properties(solver);
            for k1 = 1:numel(prop)
                p = prop(k1);
                try
                    this.addprop(p{1});
                end
                this.(p{1}) = solver.(p{1});
            end
            this.Rtc_solver = solver;
            this.complexity = complexity;
        end

        function models = fit(this, x, idx, X, varargin)
            [models0, x_corr] = this.Rtc_solver.fit(x, idx, X, varargin{:});

            corners = varargin{1};
            
            idx = [idx{:}];
            models = [];
            for k=1:numel(models0)
                model = models0(k);
                A = model.R * X + [model.t;0];
                [f, lambda, tz] = xX_to_f_l_tz(...
                            x_corr(1,idx)-model.c(1), x_corr(2,idx)-model.c(2),...
                            A(1,idx), A(2,idx), A(3,idx), this.complexity);
                model.K = [diag([f,f]) model.c; 0 0 1];
                model.proj_params = lambda';
                model.Rt = [model.R [model.t; tz]];
                models = [models model];
            end
            if ~isempty(models)
                models = WRAP.RationalSolver.valid_models(models, corners);
                models = rmfield(models,{'c','R','t'});
            end
        end
    end
end
