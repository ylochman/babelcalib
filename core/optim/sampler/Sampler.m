classdef Sampler < handle
    properties
        min_trial_count = 100;
        max_trial_count = 1e4;
        max_num_retries = 200;        
        confidence = 0.99

        N
        G = [];
        mss = []
        pmap = [];
    end
    
    methods
        function this = Sampler(mss, G, varargin)
            [this,~] = cmp_argparse(this,varargin{:});
            this.mss = mss;
            this.G = G;
            this.N = numel(unique(G));
            freq = hist(this.G,1:this.N);

            umss = unique(this.mss);
            this.pmap = containers.Map('KeyType','double','ValueType','any');
            for k = umss
                is_good = freq >= k;
                Z = zeros(1,numel(freq));
                Z(is_good) = arrayfun(@(z) nchoosek(z, k), ...
                                      freq(is_good)); 
                this.pmap(k) = normalize(Z,'norm',1);
            end
        end

        function idx = sample(this, varargin)
            idx = {};
            Gidxs = [];
            for k=this.mss
                p = this.pmap(k);
                p(Gidxs) = 0;
                Gidx = find(mnrnd(1,normalize(p,'norm',1),1));
                Gidxs = [Gidxs Gidx];
                cand_idx = find(this.G==Gidx);
                idx = {idx{:} randsample(cand_idx,k)};
            end
        end

        function trial_count = update_trial_count(this,cs)
            n = sum(this.mss);
            w = sum(cs)/numel(cs);
            trial_count = round(log(1-this.confidence)/log(1-w^n));
            trial_count = max(this.min_trial_count,min([trial_count,this.max_trial_count]));
        end
    end
end