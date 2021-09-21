classdef Solver < handle & matlab.mixin.Heterogeneous
    properties
        mss = containers.Map();
        normalization = 'diag';
        name = [];
        baselineT_rgn = 0;
    end

    methods
        function this = Solver(mss, varargin)
            this = cmp_argparse(this, varargin{:});
            if nargin > 0
                this.mss = mss;
            end
        end
            
        function flag = is_sample_good(this, meas, idx, varargin)
            flag = true;

            % Regions
            if idx.isKey('rgn') && ~isempty(idx('rgn'))
                x = meas('rgn');
                idx = idx('rgn');
                x = x(:,[idx{:}]);
                mss = this.mss('rgn');

                x3 = reshape(x,3,[]);
                [c,ia,ib] = uniquetol(x3',1e-3,'ByRows',true);
                flag1 = numel(ia) == size(x3,2);
                flag2 = numel(unique([idx{:}])) == sum(mss);

                mss_cum = cumsum(mss);
                res = arrayfun(@(k) [ones(1,mss(k)-1);2:mss(k)], 1:numel(mss),'UniformOutput',0);
                res(2:end) = arrayfun(@(k) mss_cum(k-1)+res{k}, 2:numel(mss),'UniformOutput',0);
                cspond = [res{:}];
                flag3 = all(LAF.filter_by_baseline(x, cspond, this.baselineT_rgn));
                flag = flag & flag1 & flag2 & flag3;
            end
        end

        function flag = is_valid_real(this, M)
            flag = false(1,numel(M));
            qflag = true(1,numel(M));
            for k = 1:numel(M)
                if isfield(M, 'q')
                    qflag(k) = isreal(M(k).q) & all(~isnan(M(k).q));
                end
                flag(k) = isreal(M(k).l) & all(~isnan(M(k).l));
            end
            flag = flag & qflag;
        end

        function flag = is_model_good(this, varargin)
            flag = true;
        end
    end
end