clear; clc

nsample = 100
angle  = linspace(-pi,pi,nsample);
radius = linspace(0,100,nsample);
RADIUS = 60;
X = [];
Y = [];
n = 0
for i = 1:nsample
    for j = 1:nsample
        n = n + 1;
        Pang = cos(angle(i));
        Prad = 1-exp(-radius(j)./RADIUS);
        
        X(n) = radius(j)*sin(angle(i));
        Y(n) = radius(j)*cos(angle(i));
        output(n) = ( Pang + Prad - .5 );
    end
end

plot3(X,Y,output, '.')