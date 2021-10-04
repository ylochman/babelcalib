classdef Solver < handle & matlab.mixin.Heterogeneous
    properties
        mss = containers.Map();
        name = [];
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
        end

        function flag = is_model_good(this, varargin)
            flag = true;
        end
    end
end