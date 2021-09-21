classdef Sampler < handle
    properties
        min_trial_count = 100;
        max_trial_count = 1e4;
        max_num_retries = 200;        
        confidence = 0.99

        N
        N2
        G = [];
        G2 = [];
        mss = []
        pmap = [];
    end
    
    methods
        function this = Sampler(mss, G, G2, varargin)
            [this,~] = cmp_argparse(this,varargin{:});
            this.mss = mss;
            this.G = G;
            this.N = numel(unique(G));
            freq = hist(this.G,1:this.N);
            if nargin > 2
                this.G2 = G2;
                this.N2 = numel(unique(G2));
                freq2 = arrayfun(@(k) numel(unique(this.G2(this.G==k))),1:this.N);
            end

            umss = unique(this.mss);
            this.pmap = containers.Map('KeyType','double','ValueType','any');
            for k = umss
                if isempty(this.G2)
                    is_good = freq >= k;
                else
                    is_good = freq >= k & freq2 >= k;
                end
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
                if isempty(this.G2)
                    idx = {idx{:} randsample(cand_idx,k)};
                else
                    G2idxs = randsample(unique(this.G2(cand_idx)),k);
                    idx = {idx{:} arrayfun(@(G2idx) randsample2(find(this.G==Gidx & this.G2==G2idx),1), G2idxs)};
                end
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

function el = randsample2(vec,k)
    el = vec(randsample(numel(vec),k));
end