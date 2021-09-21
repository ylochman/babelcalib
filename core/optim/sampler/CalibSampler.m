classdef CalibSampler < handle
    properties
        min_trial_count = 1e2;
        max_trial_count = 1e4;
        max_num_retries = 2e2;
        confidence = 0.99

        num_imgs
        num_boards
        img_p
        board_p

        samplers
        mss_total_n = 0;

        meas_types = [];
    end
    methods
        function this = CalibSampler(mss, groups, groups2, varargin)
            [this,~] = cmp_argparse(this, varargin{:});
            this.num_imgs = numel(mss);
            this.num_boards = numel(first_el(groups(first_el(groups.keys))));
            this.board_p = ones(this.num_imgs,this.num_boards) * NaN;

            % assert(numel(mss{n}.keys)==1)
            this.meas_types = arrayfun(@(n) first_el(mss{n}.keys),...
                                        1:this.num_imgs, 'UniformOutput', 0);
            for n=1:this.num_imgs
                G = groups(this.meas_types{n});
                G2 = [];
                if isKey(groups2, this.meas_types{n})
                    G2 = groups2(this.meas_types{n});
                end
                mss_ = mss{n}(this.meas_types{n});
                N = numel(mss_);
                freq = cellfun(@(x) hist(x,1:max(x)),...
                                    G{n},'UniformOutput', 0);
                sfreq = cellfun(@(x) sort(x,'descend'), freq,...
                                                'UniformOutput',0);
                is_valid = cellfun(@(x) (numel(x) >= N) & all(mss_ <= x(1:min(numel(x),N))), sfreq);
                if ~isempty(G2)
                    sfreq2 = arrayfun(@(k) sort(arrayfun(@(p) numel(unique(G2{n}{k}(G{n}{k}==p))),1:max(G{n}{k})),'descend'), 1:this.num_boards,'UniformOutput',0);
                    is_valid = is_valid & cellfun(@(x) (numel(x) >= N) & all(mss_ <= x(1:min(numel(x),N))), sfreq2);
                end
                assert(any(is_valid),['image ' n ' does not have enough measurements'])
                this.mss_total_n = this.mss_total_n + ...
                                    sum(mss_) * 2 * this.num_boards;
                for k=1:this.num_boards
                    this.board_p(n,k) = arrayfun(@(y) sum(arrayfun(@(x) nchoosek(x,y), freq{k}(freq{k}>=y))), mss_);
                    if ~isempty(G2)
                        this.samplers{n}{k} = Sampler(mss_,...
                                                  G{n}{k},...
                                                  G2{n}{k});
                    else
                        this.samplers{n}{k} = Sampler(mss_,...
                                                  G{n}{k});
                    end
                end
                this.board_p(n,~is_valid) = 0;
            end
            nonzero = ~~sum(this.board_p',1);
            this.board_p(nonzero,:) = this.board_p(nonzero,:)./ sum(this.board_p(nonzero,:),2);
        end
        
        function idx = sample(this, meas)
            img_idxs = this.sample_imgs();
            board_idxs = this.sample_boards(img_idxs);
            for n=1:this.num_imgs
                x = meas(this.meas_types{n});
                idxs{n} = ...
                this.samplers{img_idxs(n)}{board_idxs(n)}.sample(x{img_idxs(n)}{board_idxs(n)});
            end
            % x = meas(meas_type);
            % idxs = arrayfun(@(n)...
            %     this.samplers{img_idxs(n)}{board_idxs(n)}.sample(x{img_idxs(n)}{board_idxs(n)}),...
            %     1:this.num_imgs, 'UniformOutput', 0);
            idx = cell2struct(idxs, 'x');
            img_idxs = num2cell(img_idxs);
            [idx(:).img] = deal(img_idxs{:});
            board_idxs = num2cell(board_idxs);
            [idx(:).board] = deal(board_idxs{:});
        end

        function img_idxs = sample_imgs(this)
            img_idxs = 1:this.num_imgs;
            % img_idxs = randperm(this.num_imgs);
        end

        function board_idxs = sample_boards(this, img_idxs)
            % board_idxs = ones(1,this.num_imgs) * 3;
            res = mnrnd(1,this.board_p(img_idxs,:));
            board_idxs = rem(find(res')-1,this.num_boards)+1;
            board_idxs = reshape(board_idxs,1,[]);
        end
        

        function trial_count = update_trial_count(this, cs)
            w = sum(cs) / numel(cs);
            trial_count = round(log(1-this.confidence) / ...
                                        log(1-max(eps,w^this.mss_total_n)));
            trial_count = max(this.min_trial_count,...
                          min([trial_count, this.max_trial_count]));
        end
    end
end 