classdef x7X_to_Rt_c < WRAP.Solver
    properties
        min_N = 7;
        N = [];
        CORRECTION = 1;
    end
    
    methods
        function this = x7X_to_Rt_c(N)
            mss = containers.Map();
            mss('pt') = [N];
            this = this@WRAP.Solver(mss);
            assert(N > this.min_N);
            this.N = N;
        end
        
        function [models, x_corr] = fit(this, x, idx, X, varargin)
            X(3,:) = 1;
            idx = [idx{:}];
            if this.CORRECTION
                F1=x7X_to_Rt_c(x(1,idx), x(2,idx), X(1,idx), X(2,idx));
                R = [];
                t = [];
                c = [];
                F = [];
                for k=1:size(F1,3)
                    [F0,c0,~,R0,t0] = sampson_to_Rt_c(F1(:,:,k), x(:,idx), X(:,idx));
                    F = dstack(F,repmat(F0,1,1,size(t0,2)));
                    R = dstack(R,R0);
                    t = hstack(t,t0);
                    c = hstack(c,c0);
                end
            else
                [F,c,~,R,t]= x7X_to_Rt_c(x(1,idx), x(2,idx), X(1,idx), X(2,idx));
                F = dstack(F,repmat(F,1,1,size(t,2)));
            end
            if this.CORRECTION
                x0 = x(1,:);
                y0 = x(2,:);
                for k=1:size(F,3)
                    % Corner correction
                    a = F(:,:,k) * X;
                    x_corr(1,:,k) = (a(2,:).^2 .* x0 - a(1,:).*a(2,:) .* y0 - a(3,:).*a(1,:))./...
                    (a(1,:).^2 + a(2,:).^2);
                    x_corr(2,:,k) = (a(1,:).^2 .* y0 - a(1,:).*a(2,:) .* x0 - a(3,:).*a(2,:))./...
                    (a(1,:).^2 + a(2,:).^2);
                    x_corr(3,:,k) = x(3,:);
                end
            end
            R_array = arrayfun(@(x) reshape(x{1},3,3), mat2cell(reshape(R,9,size(R,3)),9,ones(1,size(R,3))),'UniformOutput',0);
            models = struct('c',mat2cell(c,2,ones(1,size(c,2))),...
                            'R',R_array,...
                            't',mat2cell(t,2,ones(1,size(t,2))));
            models = WRAP.x7X_to_Rt_c.valid_models(models);
        end
    end
    methods(Static)
        function flag = valid_proj_center(c, varargin)
            flag = all(abs(imag(c)) < 1e-9) &...
                   all(abs(real(c)) < 0.5);
        end

        function [models, flag] = valid_models(models, varargin)
            flag = arrayfun(@(m) WRAP.x7X_to_Rt_c.valid_proj_center(reshape(m.c,1,[])), models);
            models = models(flag);
        end
    end
end
