function [pos, vel] = IntKalman(z)
%
%
persistent A H Q R
persistent x P
persistent firstRun


if isempty(firstRun)
    firstRun = 1;
    

    dt = 0.1;
    A = [ 1 dt;
          0  1 ];
    H = [ 0 1 ];

    Q = [ 1 0;
          0 3 ];
    R = 10;

    x = [0 20 ]';
    P = 5*eye(2);
end


xp = A*x;                        % 추정값 예측
Pp = A*P*A' + Q;                 % 오차 공분산 예측

K = Pp*H'*inv(H*Pp*H' + R);      % 칼만 이득 계산
disp(H*Pp*H');

x = xp + K*(z - H*xp);           % 추정값 계산
P = Pp - K*H*Pp;                 % 오차 공분산 계산


pos = x(1);
vel = x(2);
