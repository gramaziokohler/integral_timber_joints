
clear all


function retval = data_pre_process (result, gear_ratio, flip_y)
  if(flip_y)
    result(:,2) = result(:,2) * -1 ;
  endif
  result = [result , result(:,2) / gear_ratio];
  retval = result;
endfunction

function plot_motor_voltage( gear_ratio,gear_ratio_string, flip_y)
  
  %gear_ratio = 139;
  encoder_steps = 44;
  %gear_ratio_string = "139";

  eval(strcat("results_", gear_ratio_string));

  data1 = data_pre_process (eval(strcat("result_555_", gear_ratio_string , "_111")), gear_ratio * encoder_steps, flip_y);
  data2 = data_pre_process (eval(strcat("result_555_", gear_ratio_string , "_126")), gear_ratio * encoder_steps, flip_y);
  data3 = data_pre_process (eval(strcat("result_555_", gear_ratio_string , "_148")), gear_ratio * encoder_steps, flip_y);
  data4 = data_pre_process (eval(strcat("result_555_", gear_ratio_string , "_LPO")), gear_ratio * encoder_steps, flip_y);
  data5 = data_pre_process (eval(strcat("result_555_", gear_ratio_string , "_168")), gear_ratio * encoder_steps, flip_y);

  figure(gear_ratio);
  hold on;
  plot(data1(:,1),data1(:,3),"r+-");
  plot(data2(:,1),data2(:,3),"g+-");
  plot(data3(:,1),data3(:,3),"b+-");
  plot(data4(:,1),data4(:,3),"o+-");
  plot(data5(:,1),data5(:,3),"o+-");


  legend_labels = {"11.1V", "12.6V", "14.8V", "4C LiPo", "16.8V"};
  legend (legend_labels, "location", "southeast");
  xlim([-255,255])
  ylim([-2.8,2.8])
  xlabel ("PWM Value");
  ylabel ("Rotation speed at gearbox output [rev/s]");
  title (strcat("Motor 555 with 1:", gear_ratio_string, " gearbox. Speed vs PWM Characteristic."));
  grid on;
  saveas(gear_ratio,strcat("01_Motor_Voltage_", gear_ratio_string, ".jpg")); 
endfunction

plot_motor_voltage(51,"051", true);
plot_motor_voltage(100,"100", false);
plot_motor_voltage(139,"139", false);