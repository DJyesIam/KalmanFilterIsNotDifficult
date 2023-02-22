clear all


dt = 0.1;
t = 0:dt:10;

Nsamples = length(t);

Xsaved = zeros(Nsamples, 2);
Zsaved = zeros(Nsamples, 1);

for k=1:Nsamples
    z = GetVel();
    [pos, vel] = IntKalman(z);
    % 위치와 속도 추정

    Xsaved(k, :) = [pos vel];
    Zsaved(k) = z;
end


figure
hold on

plot(t, Xsaved(:, 1))
xlabel('Time [sec]')
ylabel('Position [m]')

figure
hold on
plot(t, Zsaved(:), 'r.')
plot(t, Xsaved(:, 2))
xlabel('Time [sec]')
ylabel('Velocity [m/s]')