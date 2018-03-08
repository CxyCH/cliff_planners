%% Thing
bags = ["/home/ksatyaki/workspace/data_paper/rrtstar_bags/";
  "/home/ksatyaki/workspace/data_paper/upstream_bags/";
  "/home/ksatyaki/workspace/data_paper/cliff2_bags/";
  "/home/ksatyaki/workspace/data_paper/cliff_bags2/"];

% bags = ["/home/ksatyaki/workspace/data_paper/more_bags/cliff/";
%   "/home/ksatyaki/workspace/data_paper/more_bags/upstream/"];
Algorithms = ["A";
  "B"
  "C"
  "D"];

timesL = [];
timesF = [];
grpF = [];
grpL = [];

attr = 5;

maps = ["map1a", "map1c", "map2", "map3a", "map3b", "map4", "map5a", "map5b"];
%maps = ["intersect"];

mapnames = ["DiffDirections";
  "DiffSpeeds";
  "DiffQ";
  "DiffVarSpeed";
  "DiffVarDir";
  "DiffIntensity";
  "DiffPQ1";
  "DiffPQ2"];
%mapnames = maps;
tL = [];
tF = [];
gL = [];
gF = [];
  
for j=1:8
  timesL = [];
  timesF = [];
  grpL = [];
  grpF = [];
  for i=1:4
    fileName = bags(i);
    strcat(fileName, maps(j), '-pm-last.txt')
    pm_last = importdata(strcat(fileName, maps(j), '-pm-last.txt'));
    pm_last = str2num(char(pm_last(:,1)));
    
    strcat(fileName, maps(j), '-pm-first.txt')
    pm_first = importdata(strcat(fileName, maps(j), '-pm-first.txt'));
    pm_first = str2num(char(pm_first(:,1)));
    
    timesL = [timesL; pm_last(:,attr)];
    timesF = [timesF; pm_first(:,attr)];
    grpL = [grpL; repmat(Algorithms(i), size(pm_last(:,attr)))];
    grpF = [grpF; repmat(Algorithms(i), size(pm_first(:,attr)))];
    
    tL = [tL; pm_last(:,attr)];
    tF = [tF; pm_first(:,attr)];
    gL = [gL; repmat(Algorithms(i),size(pm_last(:,attr)))];
    gF = [gF; repmat(Algorithms(i),size(pm_first(:,attr)))];
%     figure(1);
%     subplot(2,4,j);
%     hold on;
%     errorbar(i, mean(pm_last(:,attr)), std(pm_last(:,attr)), 'gx');
%     
%     figure(3);
%     subplot(2,4,j);
%     hold on;
%     errorbar(i, mean(pm_first(:,attr)), std(pm_first(:,attr)), 'gx');
    
  end
  
  f(1,j) = figure;
  %subplot(2,4,j);
  boxplot(timesL, grpL);
  title(mapnames(j));
  ylabel("Planning time (s)");
  axis([0 5 -0.5 50])
  grid;
  l = legend();
  l.Visible = 'off';
    
  
  figure(2);
  subplot(2,4,j), title(mapnames(j));
  grid;
  boxplot(timesL, grpL);
  
end

Fig1 = figure();
boxplot(tF, gF);
title("First");
ylabel("Planning time (s)");
%axis([0 6 -0.1 2])
grid;
l = legend();
l.Visible = 'off';


Fig2 = figure();
boxplot(tL, gL);
title("Latest solution");
ylabel("Nodes in tree");
%axis([0 6 -0.1 20])
grid;
l = legend();
l.Visible = 'off';

%%
for j=1:8
  figure(f(1,j));
  matlab2tikz(['/home/ksatyaki/Documents/Notes/cliffhrrtx/tikz/', char(mapnames(j)), '.tex']);
end
  