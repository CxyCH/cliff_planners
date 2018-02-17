
%%
[map1aF map1aL map1aS] = read_data_plot('map1a');
%
[map1cF map1cL map1cS] = read_data_plot('map1c');
%
[map2F map2L map2S] = read_data_plot('map2');
%
[map3aF map3aL map3aS] = read_data_plot('map3a');
%
[map3bF map3bL map3bS] = read_data_plot('map3b');
%
[map4F map4L map4S] = read_data_plot('map4');

[map5aF map5aL map5aS] = read_data_plot('map5a');

[map5bF map5bL map5bS] = read_data_plot('map5b');

%
ResultsL = [map1aL; map1cL; map2L; map3aL; map3bL; map4L; map5aL; map5bL]

ResultsF = [map1aF; map1cF; map2F; map3aF; map3bF; map4F; map5aF; map5bF]

fileIDF = fopen('dataF.txt','w');
fileIDL = fopen('dataL.txt','w');

for j=1:8
  for i=1:5
    fprintf(fileIDL, ' & %0.2f $\\pm$ %0.2f', ResultsL((j-1)*5 + i,1), ResultsL((j-1)*5 + i,2));
    fprintf(fileIDF, ' & %0.2f $\\pm$ %0.2f', ResultsF((j-1)*5 + i,1), ResultsF((j-1)*5 + i,2));
  end
  fprintf(fileIDF, '\\\\  \n');
  fprintf(fileIDL, '\\\\  \n');
end
  
mean([map1aS, map1cS, map2S, map3aS, map3bS, map4S])
