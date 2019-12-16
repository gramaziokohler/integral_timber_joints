results_02;
m = result_0p0400_0p8000_0p0002_3000_8000_12000

avgWindowSize = 15; 
minmaxWindowSize = 15; 
b = (1/avgWindowSize)*ones(1,avgWindowSize);
a = 1;


x = m(:,1);
unfiltered = m(:,4);
max = movmax(m(:,4),minmaxWindowSize);
min = movmin(m(:,4),minmaxWindowSize);
filtered = filter(b,a,unfiltered);

%Plot the minmax range
rangeX = [x,fliplr(x)];
rangeY = [min,fliplr(max)];
col = [1 0.5 0.5];
fill (rangeX,rangeY,col);
hold on;
%plot(x,unfiltered)
%hold on
plot(x,filtered)
plot(x,max)
plot(x,min)
%legend('Input Data','Filtered Data','Maxed Data')