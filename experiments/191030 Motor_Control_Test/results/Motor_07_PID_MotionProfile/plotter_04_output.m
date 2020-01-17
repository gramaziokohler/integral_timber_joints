close all
clear
results_04;
pkg load financial

name_of_plot = "result_04_output.png"
xsize_of_plot = 1600
ysize_of_plot = 1200
names = {
"result_0p0100_0p0400_0p0001_2000_2",
"result_0p0200_0p0400_0p0001_2000_2",
"result_0p0400_0p0400_0p0001_2000_2",
"result_0p0800_0p0400_0p0001_2000_2",
} 

function plot_averaged(m)
  m = [m(:,1),movavg(m(:,3)-m(:,2),10,10)]
  %m = [m(:,1), m(:,3)-m(:,2)];
  plot(m(:, 1)/1000,m(:, 2));
endfunction

function plot_direct(m)
  m = [m(:,1), m(:,3)-m(:,2)];
  plot(m(:, 1)/1000,m(:, 2));
endfunction

function plot_output(m)
  plot(m(:, 1)/1000,m(:, 4));
endfunction
hold on
figure(1, 'position',[0,0,xsize_of_plot,ysize_of_plot]);

axis ([-500,3000])

for i = 1:length(names)
  m = eval(names{i})
  plot_output(m)
end

legend_strings = strrep (names,"result_","")
legend_strings = strrep (legend_strings,"_",", ")
legend_strings = strrep (legend_strings,"p",".")

legend (legend_strings ,"location", "southeast");

hold off

grid on
xlabel ("Time [ms]")
ylabel ("Output to DCMotor.SetSpeedPercent [Ratio of full speed]");
title("Output Response in Motion Profile Position PID Control")
drawnow
print(name_of_plot,["-S",num2str(xsize_of_plot),",",num2str(ysize_of_plot)],"-F:10")