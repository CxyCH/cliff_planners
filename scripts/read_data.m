function [CostPMFirst, CostPMLast, Success] = read_data_plot(i)
load('reds.mat');
if(isinteger(i))
  filenames = ["/home/chitt/workspace/data_paper/cliff_bags/map1a";...
    "/home/chitt/workspace/data_paper/cliff_bags/map1c";...
    "/home/chitt/workspace/data_paper/cliff_bags/map2";...
    "/home/chitt/workspace/data_paper/cliff_bags/map3a";...
    "/home/chitt/workspace/data_paper/cliff_bags/map3b";...
    "/home/chitt/workspace/data_paper/cliff_bags/map4"];
  
  fileName = filenames(i);
else
  fileName = i;
end

first = importdata(strcat(fileName,'-paths-first.txt'));
pm_first = importdata(strcat(fileName, '-pm-first.txt'));
last = importdata(strcat(fileName,'-paths-last.txt'));
pm_last = importdata(strcat(fileName, '-pm-last.txt'));

img = imread('/home/chitt/workspace/cpp_ws/src/smp_ros/maps/1world.pgm');

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
%   f = str2num(char(SortedPosesFirst(i,1)));
%   l = str2num(char(SortedPosesLast(i,1)));
%   
%   figure(1);
%   plot(f(:,1),f(:,2)); hold on;
%   idx = uint16(rand()*size(f,1));
%   text(f(idx,1), f(idx,2), string(i), 'FontSize', 10);
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

CostPMLast = [median(SortedPMLast(:,1)) std(SortedPMLast(:,1));...
  mean(SortedPMLast(:,2)) std(SortedPMLast(:,2));...
  mean(SortedPMLast(:,3) - SortedPMLast(:,4)) std(SortedPMLast(:,3), SortedPMLast(:,4));...
  mean(SortedPMLast(:,4)) std(SortedPMLast(:,4));...
  mean(SortedPMLast(:,5)) std(SortedPMLast(:,5))];

Success = size(last,1) / 50;
% figure(1);
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