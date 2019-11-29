clear
raw_data = csvread ("results.csv")

data_1000 = []
data_2000 = []
data_3000 = []
data_4000 = []
data_5000 = []
data_6000 = []
data_7000 = []

gear_ratio = 51

for data_line = raw_data'
  
  if (data_line(2) != gear_ratio)
    continue
  endif
  
  if (data_line(4) == 1000)
    data_1000 = [data_1000; data_line']
  endif
  if (data_line(4) == 2000)
    data_2000 = [data_2000; data_line']
  endif
  if (data_line(4) == 3000)
    data_3000 = [data_3000; data_line']
  endif
  if (data_line(4) == 4000)
    data_4000 = [data_4000; data_line']
  endif
  if (data_line(4) == 5000)
    data_5000 = [data_5000; data_line']
  endif
  if (data_line(4) == 6000)
    data_6000 = [data_6000; data_line']
  endif
  if (data_line(4) == 7000)
    data_7000 = [data_7000; data_line']
  endif
  
endfor

figure(1)
plot (
data_1000(:, 3)/1000,data_1000(:, 6),"-x",
data_2000(:, 3)/1000,data_2000(:, 6),"-x",
data_3000(:, 3)/1000,data_3000(:, 6),"-x",
data_4000(:, 3)/1000,data_4000(:, 6),"-x",
data_5000(:, 3)/1000,data_5000(:, 6),"-x",
data_6000(:, 3)/1000,data_6000(:, 6),"-x",
data_7000(:, 3)/1000,data_7000(:, 6),"-x"

)

xlabel ("Voltage [V]")
ylabel ("Force [N]");
legend ("1A","2A","3A","4A","5A","6A","7A", "location", "northwest");
title("Screw Pull Force vs Voltage at different limiting current (Motor 555 1:51)")
grid on
saveas(1,"plot_torque_current.jpg")

