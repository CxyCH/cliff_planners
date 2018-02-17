function heatmap_plot(i)

filenames = ["/home/chitt/workspace/data_paper/upstream_bags/map1a";...
  "/home/chitt/workspace/data_paper/upstream_bags/map1c";...
  "/home/chitt/workspace/data_paper/upstream_bags/map2";...
  "/home/chitt/workspace/data_paper/upstream_bags/map3a";...
  "/home/chitt/workspace/data_paper/upstream_bags/map3b";...
  "/home/chitt/workspace/data_paper/upstream_bags/map4";...
  "/home/chitt/workspace/data_paper/upstream_bags/map5a";...
  "/home/chitt/workspace/data_paper/upstream_bags/map5b"];
load('reds.mat')

if(isinteger(i))
  fileName = i;
else 
  fileName = filenames(i);
end

last = importdata(strcat(fileName,'-paths-last.txt'));
img = imread('/home/chitt/workspace/cpp_ws/src/smp_ros/maps/1world.pgm');

x_max = 5;
y_max = 10;

resolution = 0.25;

% Compute heatmaps for paths
img1 = zeros(y_max/resolution,x_max/resolution);
for i = 1:size(last,1)
  l = str2num(char(last(i,1)));
  img1 = heatmap1(l, resolution, img1);
end

maxVal = max(img1(:));
%img1 = 255*img1./(maxVal);
img1 = uint8(img1);

img = imresize(img,0.05/resolution, 'nearest');
img1 = imcomplement(img) + img1;


figure(1);
hold on;
axis image;
plotIm = image([0,x_max],[0,y_max], img1);
set(gca,'ydir','normal');

colormap(reds/255);

end