function map = heatmap1(path, resolution, maps)
%map = uint8(zeros(size(maps)) * 255);
for i=1:size(path,1)
  maps(round(path(i,2)/resolution), round(path(i,1)/resolution)) = ...
  maps(round(path(i,2)/resolution), round(path(i,1)/resolution)) + 10;
end
map = maps;
end