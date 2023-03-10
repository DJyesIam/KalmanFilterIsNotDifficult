function z = GetVel()
%
%
persistent Velp Posp


if isempty(Posp)
    Posp = 0;
    Velp = 80;
end

dt = 0.1;

v = 0 + 10*randn;                       % 측정 잡음 

PosP = Posp + Velp*dt;                  % true position
Velp = 80 + v;                          % true speed

z = Velp;