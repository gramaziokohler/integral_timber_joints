clear
results

plot (
result_00(:,1),result_00(:,2),"+-",
result_15(:,1),result_15(:,2),"+-",
result_30(:,1),result_30(:,2),"+-"
)
axis ([-100, 100, -4200,4200]);
grid
xlabel ("`DCMotor.setSpeedPercentage(float speedPercent) [%]")
ylabel ("Speed [step/s]");
legend ("Deadband 0", "Deadband 15", "Deadband 30", "location", "southeast");
title("Speed over setSpeedPercentage value at 12V for different deadband")
print -djpg result.jpg

