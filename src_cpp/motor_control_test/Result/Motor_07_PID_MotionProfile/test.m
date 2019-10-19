pkg load financial
m = result_10_10_0_1000_2000
m = [m,movavg(m(:,3)-m(:,2),10,10)]
plot(
m(:,1),m(:,5),
m(:,1),m(:,3)-m(:,2)
)