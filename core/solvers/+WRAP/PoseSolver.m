classdef PoseSolver < WRAP.Solver
    methods
        function this = PoseSolver(varargin)
            mss = containers.Map();
            mss('pt') = [4];
            this = this@WRAP.Solver(mss, varargin{:});
        end

        function model = fit(this, x, idx, X)
            model = [];
            [R, t] = p3p_nakano_bmvc2019(x(:,1:3)./vecnorm(x(:,1:3)), X(:,1:3));
            if ~isempty(R)
                if ndims(R)==3
                    s = size(R);
                    for k=1:s(3)
                        X04 = R(:,:,k) * X(:,4) + t(:,k);
                        d(k) = vecnorm(x(:,4) * X04(1)/x(1,4) - X04);
                    end
                    [~,k]=min(d);
                    model.Rt = [R(:,:,k) t(:,k)];
                else
                    model.Rt = [R t];
                end
            end
        end

        function flag = is_sample_good(this, x, idx, X, varargin)
            keyboard
            flag = true;
        end

        function flag = is_model_good(this, varargin)
            flag = true;
        end
    end
end
