result

plot (result(:,1),result(:,2),"+-")
axis ([0, 14, 0, 1.6]);
xlabel ("Voltage [V]")
ylabel ("Speed [rev/s]");
%legend ("I");
title("Speed over Volage Relationship")
print -djpg result.jpg