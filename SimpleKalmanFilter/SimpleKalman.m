function [volt, Px, K] = SimpleKalman(z)
%
%
persistent A H Q R
persistent x P
persistent firstRun


if isempty(firstRun)
    A = 1;
    H = 1;

    Q = 0;
    R = 4;
% 시스템 모델 변수 초기화

    x = 14;                      
    P = 6;                 
% 초기 예측값 지정

    firstRun = 1;
end


xp = A*x;                        % 추정값 예측
Pp = A*P*A' + Q;                 % 오차 공분산 예측

K = Pp*H'*inv(H*Pp*H' + R);      % 칼만 이득 계산

x = xp + K*(z - H*xp);           % 추정값 계산
P = Pp - K*H*Pp;                 % 오차 공분산 계산

volt = x;                        % 추정값 반환
Px = P;

