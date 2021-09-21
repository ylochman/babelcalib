function detections = loadORPC(fn)
  % get metadata
  fid=fopen(fn); 
  meta = textscan(fid, '%s%[^\n]', 5, 'delimiter', ': ', 'collectoutput', 1);
  fclose(fid);
  detections.fn = fn;
  detections.width = str2double(meta{1}{2,2});
  detections.height = str2double(meta{1}{3,2});
  detections.num_corners = str2double(meta{1}{4,2});

  if detections.num_corners == 0
    return;
  end

  % get detections
  fid=fopen(fn); 
  detections.pts = cell2mat(textscan(fid, '%f%f%f%f%f', 'delimiter', ',', 'headerlines', 5, 'collectoutput', 1));
  fclose(fid);

  if detections.num_corners ~= size(detections.pts,1)
    error('Invalid file, number of corners != number of read detections');
  end
end