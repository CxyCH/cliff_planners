function [CostPMFirst, CostPMLast, Success] = read_data_plot(i)
%i = "/home/ksatyaki/workspace/data_paper/more_bags/cliff/intersect"
load('reds.mat');
if(isinteger(i))
  filenames = ["/home/ksatyaki/workspace/data_paper/cliff_bags2/map1a";...
    "/home/ksatyaki/workspace/data_paper/cliff_bags2/map1c";...
    "/home/ksatyaki/workspace/data_paper/cliff_bags2/map2";...
    "/home/ksatyaki/workspace/data_paper/cliff_bags2/map3a";...
    "/home/ksatyaki/workspace/data_paper/cliff_bags2/map3b";...
    "/home/ksatyaki/workspace/data_paper/cliff_bags2/map4"];
  
  fileName = filenames(i);
else
  fileName = i;
end

first = importdata(strcat(fileName,'-paths-first.txt'));
pm_first = importdata(strcat(fileName, '-pm-first.txt'));
last = importdata(strcat(fileName,'-paths-last.txt'));
pm_last = importdata(strcat(fileName, '-pm-last.txt'));

%img = imread('/home/ksatyaki/workspace/cpp_ws/src/smp_ros/maps/1world.pgm');
%img = imread('/home/ksatyaki/workspace/cpp_ws/src/smp_ros/maps/intersect.pgm');
%img = flipud(img);
%load('/home/ksatyaki/workspace/matlab/cs_toolbox/cs_toolbox_temporal/intersect.mat');
%DM.PlotMapDirectionRich(0, 0.5);

% figure(1);
% %subplot(121),
% image([0,5],[0,10], img);
% set(gca,'ydir','normal');
% axis image;
% hold on;
% 
% figure(2);
% %subplot(121), 
% image([0,5],[0,10], img);
% set(gca,'ydir','normal');
% axis image;
% hold on;

legend_str = [" "];
legend_str_last = repmat(legend_str , 8, 1);
legend_str_first = repmat(legend_str , 8, 1);

% Store in a temp variable
burp_last = str2num(char(pm_last(:,1)));  
burp_first = str2num(char(pm_first(:,1)));  

% Sort the costs
[~, IndicesLast] = sort(burp_last(:,2));
[~, IndicesFirst] = sort(burp_first(:,2));

burp_last_1 = burp_last(:,1);
burp_last_2 = burp_last(:,2);
burp_last_3 = burp_last(:,3);
burp_last_4 = burp_last(:,4);
burp_last_5 = burp_last(:,5);

burp_first_1 = burp_first(:,1);
burp_first_2 = burp_first(:,2);
burp_first_3 = burp_first(:,3);
burp_first_4 = burp_first(:,4);
burp_first_5 = burp_first(:,5);

SortedPMFirst = [burp_first_1(IndicesFirst),burp_first_2(IndicesFirst),burp_first_3(IndicesFirst), burp_first_4(IndicesFirst), burp_first_5(IndicesFirst)];
SortedPMLast = [burp_last_1(IndicesLast),burp_last_2(IndicesLast),burp_last_3(IndicesLast), burp_last_4(IndicesLast), burp_last_5(IndicesLast)];

SortedPosesFirst = first(IndicesFirst);
SortedPosesLast = last(IndicesLast);
j = 1;
% for i = 1:size(last,1)
% %   if(mod(i,5) ~= 0)
% %     continue;
% %   end
l = str2num(char(SortedPosesLast(1,1)));
%   
% figure(1);
% plot(l(:,1),l(:,2), 'g', 'LineWidth', 3); hold on;
% text(l(100,1), l(100,2), "DTC-RRT*", 'FontSize', 14);
% text(l(1,1), l(1,2), "S", 'FontSize', 14);
% text(l(end,1), l(end,2), "G", 'FontSize', 14);

%   legend_str_first(j) = strcat(string(i), ' :', string(SortedPMFirst(i,2)), ', ',  string(SortedPMFirst(i,1)), ', ', string(SortedPMFirst(i,3)), ', ', string(SortedPMFirst(i,4)), ', ', string(SortedPMFirst(i,5)));
%   
%   figure(2);
%   idx = uint16(rand()*size(l,1));
%   text(l(idx,1), l(idx,2), string(i), 'FontSize', 10);
%   plot(l(:,1),l(:,2)); hold on;
%   legend_str_last(j) = strcat(string(i), ' :', string(SortedPMLast(i,2)), ', ',  string(SortedPMLast(i,1)), ', ', string(SortedPMLast(i,3)), ', ', string(SortedPMLast(i,4)), ', ', string(SortedPMLast(i,5))); 
%   j = j + 1;
% end

CostPMFirst = [mean(SortedPMFirst(:,1)) std(SortedPMFirst(:,1));...
  mean(SortedPMFirst(:,2)) std(SortedPMFirst(:,2));...
  mean(SortedPMFirst(:,3)- SortedPMFirst(:,4)) std(SortedPMFirst(:,3) - SortedPMFirst(:,4));...
  mean(SortedPMFirst(:,4)) std(SortedPMFirst(:,4));...
  mean(SortedPMFirst(:,5)) std(SortedPMFirst(:,5))];

CostPMLast = [mean(SortedPMLast(:,1)) std(SortedPMLast(:,1));...
  mean(SortedPMLast(:,2)) std(SortedPMLast(:,2));...
  mean(SortedPMLast(:,3) - SortedPMLast(:,4)) std(SortedPMLast(:,3), SortedPMLast(:,4));...
  mean(SortedPMLast(:,4)) std(SortedPMLast(:,4));...
  mean(SortedPMLast(:,5)) std(SortedPMLast(:,5))];

Success = size(last,1) / 25;
% colormap gray;
% %subplot(122), 
% l1 = legend(legend_str_first);
% l1.FontSize = 12;
% l1.FontName = 'Inconsolata';
% l1.Location =' NorthEastOutside';
% figure(2);
% colormap gray;
% %subplot(122),
% l2 = legend(legend_str_last);
% l2.FontSize = 12;
% l2.FontName = 'Inconsolata';
% l2.Location =' NorthEastOutside';

end