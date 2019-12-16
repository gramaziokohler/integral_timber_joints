clear
raw_data = csvread ("data/results.csv")

data_3000 = []
data_4000 = []
data_5000 = []



for data_line = raw_data'
  if (data_line(2) == 3000)
    data_3000 = [data_3000; data_line']
  endif
  if (data_line(2) == 4000)
    data_4000 = [data_4000; data_line']
  endif
    if (data_line(2) == 5000)
    data_5000 = [data_5000; data_line']
  endif
endfor

figure(1)
plot (
data_3000(:, 3)/1000,data_3000(:, 5),"-x",
data_4000(:, 3)/1000,data_4000(:, 5),"-x",
data_5000(:, 3)/1000,data_5000(:, 5),"-x"

)

xlabel ("Bearing Spacing [mm]")
ylabel ("Force [N]");
legend ("3A","4A","5A","location", "northwest");
title("Clamp Jaw Pull Force vs Bearing Distance at different limiting current (Motor 555 1:51)")
grid on
saveas(1,"plot_force_bearing_distance.jpg")

