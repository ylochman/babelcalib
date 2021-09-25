function [boards, board_idxs] = get_boards_from_dsc(dsc_path, tp_path, board_idxs)
    target = CalibrationTarget(dsc_path, tp_path);
    if isempty(board_idxs)
        board_idxs = 1:numel(target.boards_);
    end
    num_boards = numel(board_idxs);
    boards = [];
    for n=1:num_boards
        b = target.boards_{board_idxs(n)};
        if ~isfield(b, 'Rt')
            boards(n).Rt = [eye(3) [0; 0; 0]];
        else
            boards(n).Rt = b.Rt;
        end;
        boards(n).X = b.pts(:,1:2)';
    end
    S = 10;
    for n=1:num_boards
        boards(n).X = boards(n).X./S;
        boards(n).Rt(:,4) = boards(n).Rt(:,4)./S;
    end
end