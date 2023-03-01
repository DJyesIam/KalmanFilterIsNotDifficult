function [ax, ay, az] = getAccel()
%
%
persistent fx fy fz
persistent k firstRun


if isempty(firstRun)
    load ArsAccel
    k = 1;

    firstRun = 1;
end

ax = fx(k);
ay = fy(k);
az = fz(k);

k = k + 1;