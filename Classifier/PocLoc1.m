function [v, om] = PocLoc1(laser_rp)

v = -0.2 + 0.009*laser_rp(36);
%om1 = -30 + laser_rp(5);
om2 = +30 - laser_rp(31);
%om = 0.01*(om1+om2)/2;
om = 0.02*(om2);