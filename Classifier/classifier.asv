function map = classifier(laser_rp,r_pose)

%%% Generate list of ranges and relative angles
laser_rth = zeros(length(laser_rp),2);
angle_increment = (2*pi)/length(laser_rp);
for index = 1:length(laser_rp)
    angle = index*angle_increment + r_pose(3);
    laser_rth(index,:) = [laser_rp(index),angle];
end

points = parse_scan(laser_rth);

map = zeros(length(laser_rp));