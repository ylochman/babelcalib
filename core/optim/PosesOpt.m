classdef PosesOpt < handle
    properties
        params_idx = []; % indexing and buffers
        Jpat = []; % sparsity pattern for the Jacobian

        reprojT = 1e-3; % max geometric distance
        max_iter = 50;

        img_cspond = [];
        boards = [];
        weight_fn = @huberWeightIRLS;

        proj_fn = @RP2.project_kb;
    end
    
    methods(Access = public)
	function this = PosesOpt(img_cspond, boards, varargin)
            this = cmp_argparse(this, varargin{:});
            this.img_cspond = img_cspond;
            this.boards = boards;
            
            [aa,bb] = ndgrid(1:3,1:max(img_cspond));
            R_idx = reshape(aa+(bb-1)*3,[],1);
            t_idx = reshape(R_idx(end)+aa+(bb-1)*3,[],1);
            
            this.params_idx = struct('R', R_idx, ...
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
            
            [dz,z0] = this.pack_all(model.Rt);
            
            [residual, dx, w] = this.calc_residual_all(dz,z0,xdX,model.K,model.proj_params);
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
            options = optimoptions(@lsqnonlin, common_params{:});
            
            calc_residual_fn = @(dz,z0,xdX,K,proj_params) this.calc_residual(dz,z0,xdX,K,proj_params);
            M = M0;
            residual = [];
            for k = 1:max(this.img_cspond)
                [dz0,z0] = this.pack(M0.Rt(:,:,k));
                [dz,~,residual_] = ...
                    lsqnonlin(calc_residual_fn,dz0,[],[],options,z0,xdX(:,this.img_cspond==k),M0.K,M0.proj_params);
                M.Rt(:,:,k) = this.unpack(dz,z0);
                residual = [residual; residual_];
            end
            
            loss = sum(residual.^2,'all');
            assert(loss <= res0.loss, ...
                   'Local optimization increased the loss.');
            
            [dz,z0] = this.pack_all(M.Rt);
            [~, dx, w] = this.calc_residual_all(dz,z0,xdX,M.K,M.proj_params);
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
        
        function [residual, dx, w] = calc_residual(this,dz,z0,xdX,K,proj_params)
            n = size(xdX,2);
            Rt = this.unpack(dz,z0);
            xp = K*this.proj_fn(Rt * xdX(4:7,:), [], proj_params);
            dx = xp(1:2,:)-xdX(1:2,:);
            w = this.weight_fn(vecnorm(dx), this.reprojT);
            residual = w.*dx;
            residual = residual(:);
        end

        function Rt = unpack(this,dz,z0)
            if isempty(dz)
                dz = zeros(size(z0));
            end
            z = z0+dz;
            R = rotationVectorToMatrix(z(1:3)); 
            t = reshape(z(4:6),3,1,[]);
            Rt = cat(2,R,t);
        end
                
        function [dz,z0,z] = pack(this,Rt)
            z0 = zeros(6,1); 
            dz = z0;
            z0(1:3) = rotationMatrixToVector(Rt(:,1:3));
            z0(4:6) = Rt(:,4);
            z = z0+dz;
        end

        function [M,lo_res] = fit_all(this,meas,M0,res0,varinput,varargin)
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
                             'MaxIter', 50, ...
                             'MaxFunEvals', 1e6};   
            if ~isempty(this.Jpat)
                common_params = cat(2,common_params, ...
                                    'JacobPattern', this.Jpat);
            end
            options = optimoptions(@lsqnonlin, ...
                                   common_params{:});

            [dz0,z0] = this.pack_all(M0.Rt);

            calc_residual_fn = @(dz,z0,xdX,K,proj_params) this.calc_residual_all(dz,z0,xdX,K,proj_params);
            [dz,~,residual] = ...
                lsqnonlin(calc_residual_fn,dz0,[],[],options,z0,xdX,M0.K,M0.proj_params);
            
            loss = sum(residual.^2,'all');
            assert(loss <= res0.loss, ...
                   'Local optimization increased the loss.');
            
            M = M0;
            M.Rt = this.unpack_all(dz,z0);
            [~, dx, w] = this.calc_residual_all(dz,z0,xdX,M.K,M.proj_params);
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
        
        function [residual, dx, w] = calc_residual_all(this,dz,z0,xdX,K,proj_params) 
            n = size(xdX,2);
            Rt = this.unpack_all(dz,z0);
            xp = nan(3,size(xdX,2));
            for k = 1:max(this.img_cspond)
                mask = this.img_cspond == k;
                xp(:,mask) = ...
                    K*this.proj_fn(Rt(:,:,k) * xdX(4:7,mask), [], proj_params);
            end
            dx = xp(1:2,:)-xdX(1:2,:);
            w = this.weight_fn(vecnorm(dx), this.reprojT);
            residual = w.*dx;
            residual = residual(:);
        end

        function Rt = unpack_all(this,dz,z0)
            if isempty(dz)
                dz = zeros(size(z0));
            end
            z = z0+dz;
            n = numel(this.params_idx.R) / 3;
            R = zeros(3,3,n);
            t = reshape(z(this.params_idx.t),3,1,[]);
            for k = 1:n
                R(:,:,k) = ...
                    rotationVectorToMatrix(z(this.params_idx.R(3*(k-1)+(1:3)))); 
            end
            Rt = cat(2,R,t);
        end
                
        function [dz,z0,z] = pack_all(this,Rt)
            n = numel(this.params_idx.R) / 3;
            z0 = zeros(this.params_idx.t(end),1); 
            dz = z0;
            for k = 1:n
                z0(this.params_idx.R(3*(k-1)+1:3*k)) = rotationMatrixToVector(Rt(:,1:3,k));
            end
            z0(this.params_idx.t) = reshape(Rt(:,4,:),[],1);
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