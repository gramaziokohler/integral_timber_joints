result

data1 = result_data;


figure(1);
hold on;
plot(data1(:,1),data1(:,2),"r--+")
plot(data1(:,1),data1(:,3),"m-+")
plot(data1(:,1),data1(:,4),"g--+")
plot(data1(:,1),data1(:,5),"b-+")
plot(data1(:,1),data1(:,6),"c-+")

legend_labels = {"1:54W PSU (16.6V10A)","1:54W LiPo (4Cell)","1:72W PSU (16.6V10A)","1:72W LiPo (4Cell)" ,"1:72W LiPo PWM0.8max"};
legend (legend_labels, "location", "southwest");

xlabel ("Speed [step/s]");
ylabel ("Pull Force [N]");
title({"Motor 555 with 1:54W and 1:72W worm gearbox. PWM Controlled Trapezoidal Motion Profile.";
"Acceleration 5000step/s, Various Speed, Various Power Source";
"Max pull force"});
grid on;
hold off
saveas(1,strcat("StoppingCondition.jpg")); 
