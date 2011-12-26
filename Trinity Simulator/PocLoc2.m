function [v, om] = PocLoc2(laser_rp)

v = -30 + min([laser_rp(1),laser_rp(36),laser_rp(35)]);
v = 0.1*atan(v);
om1 = -23 + (laser_rp(8)+laser_rp(9)+laser_rp(10))/3;
om2 = 23 - (laser_rp(26)+laser_rp(27)+laser_rp(28))/3;
om3 = -32.5 + (laser_rp(4)+laser_rp(5)+laser_rp(6))/3;
om4 = 32.5 - (laser_rp(30)+laser_rp(31)+laser_rp(32))/3;
%om = 0.02*(om1+om2)/2;
%c = v/250;
om = 0.009*(om1+om2+om3+om4)/4 + 0.5*atan(1/v)*rand(1) + 0.05*atan(laser_rp(35) - laser_rp(1));
%om = 0.02*(om2);

om_max = pi*(0.5); 
if om > om_max
    om = om_max;
end

if om < -om_max
    om = -om_max;
end

v_max = 1; 
if v > v_max
    v = v_max;
end


