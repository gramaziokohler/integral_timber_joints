close all
clear
results_02;

name_of_plot_1 = "result_02_error.png"
name_of_plot_2 = "result_02_output.png"
name_of_plot_3 = "result_02_position.png"
xsize_of_plot = 1600
ysize_of_plot = 1200
names = {
"result_0p0400_0p2000_0p0002_3000_8000_12000",
"result_0p0400_0p4000_0p0002_3000_8000_12000",
"result_0p0400_0p8000_0p0002_3000_8000_12000",

} 

function plot_error(m)
  m = [m(:,1), m(:,3)-m(:,2)];
  plot(m(:, 1)/1000,m(:, 2));
endfunction
function plot_ouput(m)
  plot(m(:, 1)/1000,m(:, 4));
endfunction


figure(1, 'position',[0,0,xsize_of_plot,ysize_of_plot]);
hold on
for i = 1:length(names)
  m = eval(names{i})
  plot_error(m)
end
hold off
legend_strings = strrep (names,"result_","")
legend_strings = strrep (legend_strings,"_",", ")
legend_strings = strrep (legend_strings,"p",".")
legend (legend_strings ,"location", "southeast");
grid on
axis ([-500,8000])
xlabel ("Time [ms]")
ylabel ("Error [step]");
title("Error Response in Motion Profile Position PID Control")
drawnow
print(name_of_plot_1,["-S",num2str(xsize_of_plot),",",num2str(ysize_of_plot)],"-F:10")

figure(2, 'position',[0,0,xsize_of_plot,ysize_of_plot]);
hold on
for i = 1:length(names)
  m = eval(names{i})
  plot_ouput(m)
end
hold off
legend_strings = strrep (names,"result_","")
legend_strings = strrep (legend_strings,"_",", ")
legend_strings = strrep (legend_strings,"p",".")
legend (legend_strings ,"location", "southeast");
grid on
axis ([-500,8000])
xlabel ("Time [ms]")
ylabel ("Output to DCMotor.SetSpeedPercent [Ratio of full speed]");
title("Output Response in Motion Profile Position PID Control")
drawnow
print(name_of_plot_2,["-S",num2str(xsize_of_plot),",",num2str(ysize_of_plot)],"-F:10")

figure(3, 'position',[0,0,xsize_of_plot,ysize_of_plot]);
hold on
for i = 1:length(names)
  m = eval(names{i})
  plot(m(:, 1)/1000,m(:, 3));
end
hold off
legend_strings = strrep (names,"result_","")
legend_strings = strrep (legend_strings,"_",", ")
legend_strings = strrep (legend_strings,"p",".")
legend (legend_strings ,"location", "southeast");
grid on
axis ([-500,8000])
xlabel ("Time [ms]")
ylabel ("Measured Position [step]");
title("Position Time Graph")
drawnow
print(name_of_plot_3,["-S",num2str(xsize_of_plot),",",num2str(ysize_of_plot)],"-F:10")
close all
