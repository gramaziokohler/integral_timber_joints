result_2805


data1 = result_0p0400_0p2000_0p0002_2805_1000_4;
data2 = result_0p0400_0p2000_0p0002_2805_2000_4;
data3 = result_0p0400_0p2000_0p0002_2805_3000_4;

figure(1);
hold on;
plot(data1(:,1),data1(:,3),"r-")
plot(data2(:,1),data2(:,3),"g-")
plot(data3(:,1),data3(:,3),"b-")
legend_labels = {"1000 step/s2", "2000 step/s2", "3000 step/s2"};
legend (legend_labels, "location", "southeast");

xlabel ("Time [ms]");
ylabel ("Position [step]");
title (strcat("Motor 555 with 1:51 gearbox. PWM Controlled Trapezoidal Motion Profile. Different Acceleration"));
grid on;
hold off
saveas(1,strcat("acceleration_m555_g51_s2805_pos.jpg")); 



figure(2);
hold on;
plot(data1(:,1),data1(:,3)-data1(:,2),"r-")
plot(data2(:,1),data2(:,3)-data2(:,2),"g-")
plot(data3(:,1),data3(:,3)-data3(:,2),"b-")
legend_labels = {"1000 step/s2", "2000 step/s2", "3000 step/s2"};
legend (legend_labels, "location", "southeast");

xlabel ("Time [ms]");
ylabel ("Error [step]");
title (strcat("Motor 555 with 1:51 gearbox. PWM Controlled Trapezoidal Motion Profile. Different Acceleration"));
grid on;
hold off
saveas(2,strcat("acceleration_m555_g51_s2805_error.jpg")); 
