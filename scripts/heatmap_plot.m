filenames = ["/home/ksatyaki/workspace/data_paper/cliff2_bags/map1a";...
  "/home/ksatyaki/workspace/data_paper/cliff2_bags/map1c";...
  "/home/ksatyaki/workspace/data_paper/cliff2_bags/map2";...
  "/home/ksatyaki/workspace/data_paper/cliff2_bags/map3a";...
  "/home/ksatyaki/workspace/data_paper/cliff2_bags/map3b";...
  "/home/ksatyaki/workspace/data_paper/cliff2_bags/map4";...
  "/home/ksatyaki/workspace/data_paper/cliff2_bags/map5a";...
  "/home/ksatyaki/workspace/data_paper/cliff2_bags/map5b"];
load('reds.mat')

mapnames = ["DiffDirections";
  "DiffSpeeds";
  "DiffQ";
  "DiffVarSpeed";
  "DiffVarDir";
  "DiffIntensity";
  "DiffPQ1";
  "DiffPQ2"];


x_max = 5.0;
y_max = 10.0;
  
resolution = 0.2;

img = imread('/home/ksatyaki/workspace/cpp_ws/src/smp_ros/maps/1world.pgm');
%img = imresize(img,49/200, 'nearest');

cb = hsv(40);
ca = winter(20);

colours = [ca(1:5,:); flipud(cb(1:22,:))];


for ii=1:8
  fileName = filenames(ii);
  last = importdata(strcat(fileName,'-paths-last.txt'));
  
  % Compute heatmaps for paths
  img1 = zeros(uint8(y_max/resolution),uint8(x_max/resolution));
  for i = 1:size(last,1)
    l = str2num(char(last(i,1)));
    img1 = heatmap1(l, resolution, img1);
  end
  
  maxVal = max(img1(:));
  %img1 = 255*img1./25;
  img1 = uint8(img1);
  %img1 = [img1(:,1:24), zeros(49,1)];
  
  img1 = (uint8(logical(img))) + imresize(img1, 4, 'cubic');
 
  
  f(1, ii) = figure;
  hold on;
  axis image;
  %colormap([0.0 0.0 0.0; 1.0 1.0 1.0; hsv(50)]);
  colormap([0.0 0.0 0.0; 1.0 1.0 1.0; colours]);
  title(mapnames(ii));
  if(ii == 8)
    colorbar;
    caxis('manual')
    caxis([-1 25]);
  end
  %caxis([0, 25]);
  l = legend();
  l.Visible = 'off';
  plotIm = image([0,x_max],[0,y_max], img1);
end

%%
for j = 1:8
  figure(f(1,j));
  matlab2tikz(['/home/ksatyaki/Documents/Notes/cliffhrrtx/conference-paper/', char(mapnames(j)), '-cliff-heat.tex']);
end
