clear all

dt = 0.2;
t = 0:dt:10;

Nsamples = length(t);

Xsaved = zeros(Nsamples, 3);
Zsaved = zeros(Nsamples, 1);

for k=1:Nsamples
    z = GetVolt();
    [volt, Cov, Kg] = SimpleKalman(z);           % 칼만 필터 함수 호출

    Xsaved(k, 1) = volt;
    Xsaved(k, 2) = Cov;
    Xsaved(k, 3) = Kg;
    Zsaved(k) = z;
end


figure
plot(t,Xsaved(:,1),'o-')
hold on
plot(t,Zsaved, 'r:*')
xlabel('Time [sec]')
ylabel('Voltage [V]')
legend('Kalman Filter', 'Measurement')

figure
plot(t,Xsaved(:,2),'o-')
xlabel('Time [sec]')
ylabel('P')

figure
plot(t,Xsaved(:,3),'o-')
xlabel('Time [sec]')
ylabel('K')