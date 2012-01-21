%%% Simple right wall follow algorithm %%%
function [v, om] = PocLoc1Ab(laser_rp)

weight1 = 0.3; %0.3;
weight2 = 0.2; %0.2;
weight3 = 0.5; %0.5;

d1 = laser_rp(26); %27
d2 = laser_rp(28); %29
d3 = laser_rp(30); %31
%d3 = laser_rp(31); % was 30

if(d1 > 40)
	d1 = 40;
end;

if(d2 > 40)
	d2 = 40;
end;

if(d2 > 40)
	d2 = 40;
end;

d = weight1*d1 + weight2*d2 + weight3*d3;
d_2 = min(laser_rp(26), min(laser_rp(27), min(laser_rp(28), min(laser_rp(29), laser_rp(30)))));
d_net = 0.85*d - 0.45*(1000/((d_2 - 26)^2));

om = 0;
d_set = 26; %30
if(d_net > d_set || laser_rp(36) < d_set)
	om = -0.2;
end
if(d_net < d_set || laser_rp(36) < d_set)
	om = 0.2;
end
%%om = -.008*(d - 30);
v = (.024*(abs(om)))*laser_rp(36);