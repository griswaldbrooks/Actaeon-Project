clear; clc;
R = linspace(0,400, 100)
T = linspace(-pi,pi,100)
I = 125;

X = zeros(1,100);
Y = zeros(1,100);
Z = zeros(1,100);
n = 0;
for r = R
    for t = T
        n = n + 1;
        a = cos( t );
        b = (r-3)^4
        
        X(n) = cos(t)*r;
        Y(n) = sin(t)*r;
        Z(n) = (a*b)+cos(t);
    end
end

plot3( X, Y, Z )


