clear
results

%First plot is error of measured speed vs programmed speed
figure (1);
plot (
result_00(:,1),(result_00(:,2)-result_00(:,1))./(result_00(:,1)+1)*100,"+-",
result_30(:,1),(result_30(:,2)-result_30(:,1))./(result_30(:,1)+1)*100,"+-"
)
axis ([-4000,4000]);
grid
xlabel ("Target Value of PID, Speed of motor[step/s]")
ylabel ("Average Error Measured [error %]");
legend ("Deadband 0", "Deadband 30", "location", "southeast");
title("Accuracy of PID at maintaining speed.")
print -djpg result_error.jpg

%Second plot is error of measured speed vs programmed speed
figure (2);
plot (
result_00(:,3),result_00(:,2),"+-",
result_30(:,3),result_30(:,2),"+-"
)
%axis ([-4000,4000]);
grid
xlabel ("Output Value of PID, DCMotor.setSpeedPercent() motor[ratio]")
ylabel ("Average Measured Speed [step/s]");
legend ("Deadband 0", "Deadband 30", "location", "southeast");
title("Output value of PID at various speed.")
print -djpg result_controlval.jpg

