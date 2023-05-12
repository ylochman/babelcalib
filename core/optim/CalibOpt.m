classdef CalibOpt < handle
    properties
        params_idx = []; % indexing and buffers
        Jpat = []; % sparsity pattern for the Jacobian

        reprojT = 5e-4; % max geometric distance
        max_iter = 50;

        img_cspond = [];
        boards = [];
        weight_fn = @huberWeightIRLS;

        num_proj_params = [];
        proj_fn = @RP2.project_kb;
    end
    
    methods(Access = public)
	function this = CalibOpt(img_cspond, boards, varargin)
            this = cmp_argparse(this, varargin{:});
            this.img_cspond = img_cspond;
            this.boards = boards;
            
            proj_params_idx = 1:this.num_proj_params;
            f_idx = proj_params_idx(end)+1;
            alpha_idx = f_idx(end)+1;
            s_idx = alpha_idx(end)+1;
            pp_idx = s_idx(end)+[1:2];
            [aa,bb] = ndgrid(1:3,1:max(img_cspond));
            R_idx = reshape(pp_idx(end)+aa+(bb-1)*3,[],1);
            t_idx = reshape(R_idx(end)+aa+(bb-1)*3,[],1);
            
            this.params_idx = struct('proj_params',proj_params_idx,...
                                     'f', f_idx, ...
                                     'alpha', alpha_idx, ...
                                     's', s_idx, ...
                                     'pp', pp_idx, ...
                                     'R', R_idx, ...
                                     't', t_idx);

            this.Jpat = this.make_Jpat();
        end
        
        function [loss, errs, ir, info] = calc_loss(this,meas,...
                                            model, varinput, varargin)
            corners = meas('corners');
            [X_, Xcspond] = GRID.board_to_world(this.boards);
            x = [corners(:).x];
            X = [];
            for n=1:numel(corners)
                X0 = arrayfun(@(c1,c2) X_(:,Xcspond(1,:)==c1 & Xcspond(2,:)==c2), corners(n).cspond(1,:),corners(n).cspond(2,:),'UniformOutput',0);
                X = [X X0{:}];
            end
            X = RP3.homogenize(X);
            xdX = [x; X];
            
            [dz,z0] = this.pack(model);
            
            [residual, dx, w] = this.calc_residual(dz,z0,xdX);
            residual = reshape(residual,2,[]);
            errs = vecnorm(residual);
            loss = sum(residual.^2,'all');
            cs = sum(dx.^2) < this.reprojT^2; % consensus set
            ir = sum(cs)/numel(cs);
            info = struct('residual', residual,...
                          'cs', cs,...
                          'dx', dx,...
                          'w', w);
        end
                        
        function [residual, dx, w] = calc_residual(this,dz,z0,xdX) 
            n = size(xdX,2);
            M = this.unpack(dz,z0);
            K = M.K;
            xp = nan(3,size(xdX,2));
            for k = 1:max(this.img_cspond)
                mask = this.img_cspond == k;
                xp(:,mask) = M.K*...
                this.proj_fn(M.Rt(:,:,k) * xdX(4:7,mask), [], M.proj_params);
            end
            dx = xp(1:2,:)-xdX(1:2,:);
            w = this.weight_fn(vecnorm(dx), this.reprojT);
            residual = w.*dx;
            residual = residual(:);
        end
        
        function [M,lo_res] = fit(this,meas,M0,res0,varinput,varargin)
            corners = meas('corners');
            [X_, Xcspond] = GRID.board_to_world(this.boards);
            x = [corners(:).x];
            X = [];
            for n=1:numel(corners)
                X0 = arrayfun(@(c1,c2) X_(:,Xcspond(1,:)==c1 & Xcspond(2,:)==c2), corners(n).cspond(1,:),corners(n).cspond(2,:),'UniformOutput',0);
                X = [X X0{:}];
            end
            X = RP3.homogenize(X);
            xdX = [x;X];

            fprintf('Starting non-linear refinement (%3.2f%% inliers).\n', res0.ir*100);

            common_params = {'Display', 'iter', ...
                             'MaxIter', this.max_iter, ...
                             'MaxFunEvals', 1e6};   
            if ~isempty(this.Jpat)
                common_params = cat(2,common_params, ...
                                    'JacobPattern', this.Jpat);
            end
            options = optimoptions(@lsqnonlin, ...
                                   common_params{:});

            [dz0,z0] = this.pack(M0);

            calc_residual_fn = @(dz,z0,xdX) this.calc_residual(dz,z0,xdX);
            [dz,~,residual] = ...
                lsqnonlin(calc_residual_fn,dz0,[],[],options,z0,xdX);
            loss = sum(residual.^2,'all');
            assert(loss <= res0.loss, ...
                   'Local optimization increased the loss.');

            M = this.unpack(dz,z0);
            [~, dx, w] = this.calc_residual(dz,z0,xdX);
            residual = reshape(residual,2,[]);
            errs = vecnorm(residual);
            cs = sum(dx.^2) < this.reprojT^2;
            ir = sum(cs)/numel(cs);

            loss_info = struct('min_model', M0, ...
                               'min_res', res0, ...
                               'residual', residual,...
                               'cs', cs, ...
                               'dx', dx,...
                               'w', w);

            lo_res = struct('residual', residual, ...
                            'errs',  errs, ...
                            'loss', loss, ...
                            'ir', ir, ...
                            'dz', dz, ...
                            'info', loss_info);
        end
        
        function M = unpack(this,dz,z0)
            if isempty(dz)
                dz = zeros(size(z0));
            end
            z = z0+dz;
            proj_params = z(this.params_idx.proj_params)';
            f = z(this.params_idx.f);
            alpha = z(this.params_idx.alpha);
            s = z(this.params_idx.s);
            pp = z(this.params_idx.pp);
            K = diag([alpha*f f 1]);
            K(1,2) = s;
            K(1:2,3) = pp;
            n = numel(this.params_idx.R) / 3;
            R = zeros(3,3,n);
            t = reshape(z(this.params_idx.t),3,1,[]);
            for k = 1:n
                R(:,:,k) = ...
                    rotationVectorToMatrix(z(this.params_idx.R(3*(k-1)+(1:3)))); 
            end
            M = struct('proj_params', proj_params, ...
                       'K', K, ...
                       'Rt', cat(2,R,t));
        end
                
        function [dz,z0,z] = pack(this,model)
            n = numel(this.params_idx.R) / 3;
            z0 = zeros(this.params_idx.t(end),1); 
            dz = z0;
            z0(this.params_idx.proj_params) = model.proj_params;
            z0(this.params_idx.f) = model.K(2,2);
            z0(this.params_idx.alpha) = model.K(1,1) / model.K(2,2);
            z0(this.params_idx.s) = model.K(1,2);
            z0(this.params_idx.pp) = model.K(1:2,3);
            for k = 1:n
                z0(this.params_idx.R(3*(k-1)+1:3*k)) = rotationMatrixToVector(model.Rt(:,1:3,k));
            end
            z0(this.params_idx.t) = reshape(model.Rt(:,4,:),[],1);
            z = z0+dz;
        end

        function Jpat = make_Jpat(this)
            img_cspond = kron(this.img_cspond, [1 1]);
            Jpat = ones(numel(img_cspond),max(this.params_idx.t));
            for k=1:max(img_cspond)
                idxs = logical(kron(1:max(img_cspond)~=k,[1 1 1]));
                Jpat(img_cspond==k,this.params_idx.R(idxs)) = 0;
                Jpat(img_cspond==k,this.params_idx.t(idxs)) = 0;
            end
        end
    end
end
