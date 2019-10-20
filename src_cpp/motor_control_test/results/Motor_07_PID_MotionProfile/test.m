pkg load financial
results
m = result_0p0030_0p0060_0p0000_2000_2
%Smooth the signal
%m = [m,movavg(m(:,3)-m(:,2),10,10)]
plot(
%m(:,1)./1000,m(:,5),
m(:,1)./1000,m(:,3)-m(:,2)
)