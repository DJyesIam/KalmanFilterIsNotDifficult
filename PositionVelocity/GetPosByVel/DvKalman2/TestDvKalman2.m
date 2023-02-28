clear all


Nsamples = 500;

Xsaved = zeros(Nsamples, 2);
Zsaved = zeros(Nsamples, 1);

for k=1:Nsamples
    z = GetSonar();
    [pos vel] = DvKalman(z);
    % 위치와 속도 추정

    Xsaved(k, :) = [pos vel];
    Zsaved(k) = z;
end

dt = 0.02;
t = 0:dt:Nsamples*dt-dt;


figure
hold on
plot(t, Zsaved(:), 'r.')
plot(t, Xsaved(:, 1))
xlabel('Time [sec]')
ylabel('Position [m]')
legend('Measurement', 'Kalman Filter')

figure
plot(t, Xsaved(:, 2))
xlabel('Time [sec]')
ylabel('Velocity [m/s]')