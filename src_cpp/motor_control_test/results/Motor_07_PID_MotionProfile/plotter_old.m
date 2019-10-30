results;
pkg load financial


names = {
"result_10_10_0_1000_2000", 
"result_15_10_0_1000_2000",
"result_20_10_0_1000_2000"
} % Comparing kp

names = {
"result_10_10_0_1000_2000",
"result_10_10_001_1000_2000",
"result_10_10_002_1000_2000"
"result_10_10_004_1000_2000"
} % Comparing kd

names = {
"result_10_10_0_1000_2000",
"result_10_20_0_1000_2000",
"result_10_30_0_1000_2000"
} % Comparing ki

names = {
"result_10_10_004_1000_2000",
"result_10_20_004_1000_2000",
"result_10_40_004_1000_2000",
"result_10_60_004_1000_2000"
} % Using new kd = 0.04, Comparing ki

function plot_averaged(m)
  m = [m(:,1),movavg(m(:,3)-m(:,2),10,10)]
  %m = [m(:,1), m(:,3)-m(:,2)];
  plot(m(:, 1)/1000,m(:, 2));
endfunction

function plot_direct(m)
  m = [m(:,1), m(:,3)-m(:,2)];
  plot(m(:, 1)/1000,m(:, 2));
endfunction

hold on
figure(1, 'position',[0,0,1000,600]);
for i = 1:length(names)
  m = eval(names{i})
  plot_averaged(m)
end
legend_strings = strrep (strrep (names,"result_",""),"_",", ")
legend (legend_strings);

hold off

grid on
xlabel ("Time [ms]")
ylabel ("Error [step]");
%legend ("Ideal Throughput (offer/thrpt = 1)","Throughput max packet size = 1000 byte","Throughput max packet size = 100 byte");
title("Step Error Over Time")
