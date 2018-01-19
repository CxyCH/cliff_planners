function [] = read_data_plot(fileName)
%fileName = './map3b';
first = importdata(strcat(fileName,'-paths-first.txt'));
pm_first = importdata(strcat(fileName, '-pm-first.txt'));
last = importdata(strcat(fileName,'-paths-last.txt'));
pm_last = importdata(strcat(fileName, '-pm-last.txt'));

img = imread('/home/chitt/workspace/cpp_ws/src/smp_ros/maps/1world.pgm');

figure(1);
subplot(121), image([0,5],[0,10], img);
set(gca,'ydir','normal');
axis image;
hold on;

figure(2);
subplot(121), image([0,5],[0,10], img);
set(gca,'ydir','normal');
axis image;
hold on;

legend_str = [" "];
legend_str_last = repmat(legend_str , size(last,1), 1);
legend_str_first = repmat(legend_str , size(first,1), 1);

% Store in a temp variable
burp_last = str2num(char(pm_last(:,1)));  
burp_first = str2num(char(pm_first(:,1)));  

% Sort the costs
[~, IndicesLast] = sort(burp_last(:,2));
[~, IndicesFirst] = sort(burp_first(:,2));

burp_last_1 = burp_last(:,1);
burp_last_2 = burp_last(:,2);
burp_last_3 = burp_last(:,3);

burp_first_1 = burp_first(:,1);
burp_first_2 = burp_first(:,2);
burp_first_3 = burp_first(:,3);


SortedPMFirst = [burp_first_1(IndicesFirst),burp_first_2(IndicesFirst),burp_first_3(IndicesFirst)];
SortedPMLast = [burp_last_1(IndicesLast),burp_last_2(IndicesLast),burp_last_3(IndicesLast)];

SortedPosesFirst = first(IndicesFirst);
SortedPosesLast = last(IndicesLast);

for i = 1:size(last,1)
  f = str2num(char(SortedPosesFirst(i,1)));
  l = str2num(char(SortedPosesLast(i,1)));
  
  figure(1);
  subplot(121), plot(f(:,1),f(:,2)); hold on;
  subplot(122), plot(f(:,1),f(:,2)); hold on;
  legend_str_first(i) = strcat(string(SortedPMFirst(i,2)), ' ,',  string(SortedPMFirst(i,1)), ' ,', string(SortedPMFirst(i,3)));
  %plot(f(:,1),f(:,2));
  
  figure(2);
  subplot(121), plot(l(:,1),l(:,2)); hold on;
  subplot(122), plot(l(:,1),l(:,2)); hold on;
  legend_str_last(i) = strcat(string(SortedPMLast(i,2)), ' ,',  string(SortedPMLast(i,1)), ' ,', string(SortedPMLast(i,3))); 
  %plot(l(:,1),l(:,2));
end

figure(1);
subplot(122), legend(legend_str_first);
figure(2);
subplot(122), legend(legend_str_last);

end