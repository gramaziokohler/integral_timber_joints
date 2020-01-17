clear all

function retval = data_pre_process (result, gear_ratio, flip_y)
  if(flip_y)
    result(:,2) = result(:,2) * -1 ;
  endif
  result = [result , result(:,2) / gear_ratio];
  retval = result;
endfunction


results_051
results_100
results_139
results_W54
results_W72
encoder_steps = 44;
legend_labels = {"1:51P", "1:100P", "1:139P", "1:54W", "1:72W"};

data1 = data_pre_process (eval("result_555_051_168"), 51 * encoder_steps, true);
data2 = data_pre_process (eval("result_555_100_168"), 100 * encoder_steps, false);
data3 = data_pre_process (eval("result_555_139_168"), 139 * encoder_steps, false);
data4 = data_pre_process (eval("result_555_W54_168"), 54 * encoder_steps, true);
data5 = data_pre_process (eval("result_555_W72_168"), 72 * encoder_steps, true);

f = figure(1);
title (strcat("Motor 555 with different gearbox. 16.8V. Speed vs PWM Characteristic."));
xlim([-255,255])
ylim([-3.5,3.5])
xlabel ("PWM Value");
ylabel ("Rotation speed at gearbox output [rev/s]");
grid on;
hold on;
plot(data1(:,1),data1(:,3),"+-");
plot(data2(:,1),data2(:,3),"+-");
plot(data3(:,1),data3(:,3),"+-");
plot(data4(:,1),data4(:,3),"+-");
plot(data5(:,1),data5(:,3),"+-");
legend (legend_labels, "location", "southeast");
hold off;
saveas(f,strcat("02_Motor_Gearbox_16V8.jpg")); 



data1 = data_pre_process (eval("result_555_051_126"), 51 * encoder_steps, true);
data2 = data_pre_process (eval("result_555_100_126"), 100 * encoder_steps, false);
data3 = data_pre_process (eval("result_555_139_126"), 139 * encoder_steps, false);
data4 = data_pre_process (eval("result_555_W54_126"), 54 * encoder_steps, true);
data5 = data_pre_process (eval("result_555_W72_126"), 72 * encoder_steps, true);

f = figure(2);
title (strcat("Motor 555 with different gearbox. 12.6V. Speed vs PWM Characteristic."));
xlim([-255,255])
ylim([-3.5,3.5])
xlabel ("PWM Value");
ylabel ("Rotation speed at gearbox output [rev/s]");
grid on;
hold on;
plot(data1(:,1),data1(:,3),"+-");
plot(data2(:,1),data2(:,3),"+-");
plot(data3(:,1),data3(:,3),"+-");
plot(data4(:,1),data4(:,3),"+-");
plot(data5(:,1),data5(:,3),"+-");

legend (legend_labels, "location", "southeast");
hold off;
saveas(f,strcat("02_Motor_Gearbox_12V6.jpg")); 

