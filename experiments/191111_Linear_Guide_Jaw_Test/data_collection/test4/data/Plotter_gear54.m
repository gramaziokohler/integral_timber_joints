clear
raw_data = csvread ("results.csv")

data_3000 = []
data_5000 = []
data_7000 = []

gear_ratio = 54

for data_line = raw_data'
  
  if (data_line(2) != gear_ratio)
    continue
  endif
  
 
  if (data_line(4) == 3000)
    data_3000 = [data_3000; data_line']
  endif
  if (data_line(4) == 5000)
    data_5000 = [data_5000; data_line']
  endif
  if (data_line(4) == 7000)
    data_7000 = [data_7000; data_line']
  endif
  
endfor

figure(1)
plot (
data_3000(:, 3)/1000,data_3000(:, 6),"-x",
data_5000(:, 3)/1000,data_5000(:, 6),"-x",
data_7000(:, 3)/1000,data_7000(:, 6),"-x"

)

xlabel ("Voltage [V]")
ylabel ("Force [N]");
legend ("3A","5A","7A", "location", "northwest");
title("Clamp Jaw Pull Force vs Voltage at different limiting current (Motor 555 worm 1:54)")
grid on
saveas(1,"plot_torque_current_54.jpg")

