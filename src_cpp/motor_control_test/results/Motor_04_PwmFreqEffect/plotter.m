clear
results

plot (
result_0030(:,1),result_0030(:,2),"+-",
result_0122(:,1),result_0122(:,2),"+-",
result_0490(:,1),result_0490(:,2),"+-",
result_3912(:,1),result_3912(:,2),"+-"
)
axis ([0, 255, 0,5000]);
grid
xlabel ("PWM Value in 255 scale [value]")
ylabel ("Speed [step/s]");
legend ("30.64 Hz", "122.55 Hz", "490.20 Hz", "3921.16Hz","location", "southeast");
title("Speed over PWM Modulation at 12V")
print -djpg result.jpg
close all
%
clear
results2

plot (
result_0030(:,1),result_0030(:,2),"+-",
result_0122(:,1),result_0122(:,2),"+-",
result_0490(:,1),result_0490(:,2),"+-",
result_3912(:,1),result_3912(:,2),"+-"
)
axis ([-100, 100, -4000,4000]);
grid
xlabel ("PWM Value in 255 scale [value]")
ylabel ("Speed [step/s]");
legend ("30.64 Hz", "122.55 Hz", "490.20 Hz", "3921.16Hz","location", "southeast");
title("Speed over PWM Modulation at 12V")
print -djpg result2.jpg