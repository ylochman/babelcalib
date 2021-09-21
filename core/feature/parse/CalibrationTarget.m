classdef CalibrationTarget < handle

  properties
    target_id_;  % name of the target
    boards_;    % definitions of the boards: global tag offset, size, feature size, tag family, tag border
  end

  methods
    function this = CalibrationTarget(dsc_file)
      this.loadFromDSCFile(dsc_file);
    end
    
    function this = loadFromDSCFile(this, dsc_file)
      % the camera ID is the filename
      [path, this.target_id_, ~] = fileparts(dsc_file);
      
      % loads dsc files, mainly board size, tag locations, 2D geometry
      this.boards_ = {}; ofs = 0;
      fid = fopen(dsc_file);
      while (~feof(fid))
        h1 = textscan(fid,'%d%d%d%f', 1, 'delimiter', ',');
        if numel(h1{1}) > 0
          h2 = textscan(fid,'%s%f', 1, 'delimiter', ',');
          board.ofs = ofs;
          board.id   = h1{1};
          board.rows = h1{2} - 1;
          if (h1{3} == 0)
            % triangular
            board.cols = h1{2} -1;
            board.type = 1;
            nodes =  board.rows*(board.cols-1)/2;
            fprintf(1, 'Reading triangular target plane %d, sz: %d x %d, %d nodes\n', board.id, board.rows, board.cols, nodes);
          else
            % rectangular
            board.cols = h1{3} - 1;
            board.type = 0;
            nodes =  board.rows*board.cols;
            fprintf(1, 'Reading rectangular target plane %d, sz: %d x %d, %d nodes\n', board.id, board.rows, board.cols, nodes);
          end;
          board.tsize = h1{4};   % feature size in mm
          board.tfam = h2{1};          
          board.tborder = h2{2};
          ofs = ofs + nodes;
          % read all the details...
          t = textscan(fid, '%d%d%d%f%f%f', nodes, 'delimiter', ',');
          % tag locations...
          board.tag_locations = find(t{1}>=0);
          board.tags = t{1}(board.tag_locations);
          % positions on the board
          board.nodes = [t(2), t(3)];
          % designed positions of points -> convert to mm
          board.pts = [t{4}*10.0, t{5}*10.0 t{6}*10.0];
          this.boards_{end+1} = board;
        else
          break;
        end;      
      end;
      fclose(fid);
      fid2 = fopen([path filesep this.target_id_ '.tp']);
      % get rid of the header
      fgetl(fid2);
      for i=1:numel(this.boards_)
        bid = fscanf(fid2, '%d', 1);
        if isempty(bid)
          break;
        end
        Rt = fscanf(fid2, '%f', [4,3])';
        for b=1:numel(this.boards_)
          if this.boards_{b}.id == bid
            this.boards_{b}.Rt = Rt;
            break;
          end;
        end
      end
      fclose(fid2);
    end
    
    function hs = show_boards(this)
      hs = zeros(numel(this.boards_));
      for i = 1:numel(this.boards_);
        b = this.boards_{i};
        fig = figure;
        h = gca(fig); hold(h, 'on');
        title(h, sprintf('Board %d', b.id));
        plot(h, b.pts(:,1), b.pts(:,2), 'rx');
        plot(h, [0, b.tsize/2], [0,0],  'r-');
        plot(h, [0,0], [0, b.tsize/2],  'g-');
        text(h, b.pts(b.tag_locations, 1) + b.tsize/2, b.pts(b.tag_locations,2) + b.tsize/2, num2str(b.tags), 'HorizontalAlignment', 'center');
        hs(i) = h;
      end
    end
    function pts = get_points(this)
      pts = [];
      for i = 1:numel(this.boards_)
        b = this.boards_{i};
        if ~isfield(b, 'Rt')
          continue;
        end;
        ptsh = b.pts'; ptsh(4,:)=1;
        ptsh = b.Rt * ptsh;
        pts = [pts , ptsh];
      end
    end
    function h = show_boards3d(this, ids, maps, sc)
      fig1 = fig;
      % we have decided at some point to remove diagonal from the
      % triangular shape, now we need to subtract 1 (type) from rows and
      % cols
      if (nargin < 2)
        ids = 1:numel(this.boards_);
      end
      if (nargin <3)
        maps = [];
      end
      if (nargin <4)
          sc = [];
      end
      c = jet(256);
      h = gca(fig1); hold(h, 'on');
      for i = ids(:)'
        b = this.boards_{i};
        rw = b.rows - b.type;       
        cl = b.cols - b.type;
        if ~isfield(b, 'Rt')
          continue;
        end
        color = repmat([0 0 1]', 1, rw*cl);
        % map points on the board to linear index
        ptsidx = sub2ind([b.rows, b.cols], b.nodes{1}'+1, b.nodes{2}'+1);
        % we only have data for these locations
        clear ptsh;
        ptsh(:, ptsidx) = b.pts'; ptsh(4,:)=1;
        ptsh = b.Rt * ptsh;
        tags = b.pts(b.tag_locations, :)';
        tags(1,:) = tags(1,:) + b.tsize/2;
        tags(2,:) = tags(2,:) + b.tsize/2;
        tags(3,:) = tags(3,:) + 0.5;
        tags(4,:) = 1;
        tags = b.Rt * tags;
        sz = 1;
        if ~isempty(maps)
          m = maps{i}(:);
          invalid_idxs = isnan(m) | m == 0;
          if isempty(sc)
              sc = [min(m), max(m)];             
          end
          % normalize data for colormap
          m = (m - sc(1))./(sc(2)-sc(1)); 
          m(m<0) = 0; m(m>1) = 1;
          m(invalid_idxs) = 0;
          color = c(floor(m*255)+1,:)';
          sz = (5+m*5).^2;
          color(:, invalid_idxs) = nan;
          ptsh(:, invalid_idxs) = nan;
          sz(invalid_idxs)= nan;
        end
        mesh(h, reshape(ptsh(1,:), rw, cl), reshape(ptsh(2,:), rw, cl), reshape(ptsh(3,:), rw, cl), reshape(color', rw, cl, 3));
        scatter3(h, ptsh(1,:), ptsh(2,:), ptsh(3,:), sz, color', 'filled');
        text(h, tags(1,:), tags(2,:), tags(3,:), num2str(b.tags), 'HorizontalAlignment', 'center');
      end
      % show coordinate system 10cm lines
      l = line([0 0 0 ; 100 0 0], [0 0 0 ; 0 100 0], [0 0 0 ; 0 0 100], 'linewidth', 3);
      set(l(1), 'color', 'r'); % x
      set(l(2), 'color', 'g'); % y
      set(l(3), 'color', 'b'); % z
      
      view(h,60,60);
      xlabel(h, 'x');
      ylabel(h, 'y');
      zlabel(h, 'z');
      axis(h, 'equal');
      axis(h, 'vis3d');
    end
  end
end % CalibrationTarget
