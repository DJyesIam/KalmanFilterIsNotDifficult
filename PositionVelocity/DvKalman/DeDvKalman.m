function [pos, vel] = DeDvKalman(z)
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
    H = [ 1 0 ];

    Q = [ 1 0;
          0 3 ];
    R = 10;

    x = [0 20 ]';
    P = 5*eye(2);
end


xp = A*x;                        % 추정값 예측
Pp = A*P*A' + Q;                 % 오차 공분산 예측

K = 1 / (Pp(1,1) + R)* [ Pp(1,1) Pp(2,1) ]';      % 칼만 이득 계산

x = xp + K*(z - H*xp);           % 추정값 계산
P = Pp - K*H*Pp;                 % 오차 공분산 계산


pos = x(1);
vel = x(2);
