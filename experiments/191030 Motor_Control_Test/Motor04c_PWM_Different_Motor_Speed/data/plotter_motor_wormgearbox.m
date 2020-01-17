clear all

function retval = data_pre_process (result, gear_ratio, flip_y)
  if(flip_y)
    result(:,2) = result(:,2) * -1 ;
  endif
  result = [result , result(:,2) / gear_ratio];
  retval = result;
endfunction


results_W54
results_W72
encoder_steps = 44;
legend_labels = {"1:54 16.8V", "1:72 16.8V","1:54 12.6V", "1:72 12.6V"};

data1 = data_pre_process (eval("result_555_W54_168"), 54 * encoder_steps, true);
data2 = data_pre_process (eval("result_555_W72_168"), 72 * encoder_steps, true);
data3 = data_pre_process (eval("result_555_W54_126"), 54 * encoder_steps, true);
data4 = data_pre_process (eval("result_555_W72_126"), 72 * encoder_steps, true);

f = figure(1);
title (strcat("Motor 555 with 1:54 and 1:72 worm gearbox. Speed vs PWM Characteristic."));
xlim([-255,255])
ylim([-3.8,3.8])
xlabel ("PWM Value");
ylabel ("Rotation speed at gearbox output [rev/s]");
grid on;
hold on;
plot(data1(:,1),data1(:,3),"r+-");
plot(data2(:,1),data2(:,3),"g+-");
plot(data3(:,1),data3(:,3),"b+-");
plot(data4(:,1),data4(:,3),"o+-");
legend (legend_labels, "location", "southeast");
hold off;
saveas(f,strcat("02_Motor_wormgearbox.jpg")); 

