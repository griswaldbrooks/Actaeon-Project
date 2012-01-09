function r_pose = classifier(laser_rp,r_pose)

%%% Generate list of ranges and relative angles
laser_rth = zeros(length(laser_rp),2);
angle_increment = (2*pi)/length(laser_rp);
for index = 1:length(laser_rp)
    angle = index*angle_increment + r_pose(3);
    laser_rth(index,:) = [laser_rp(index),angle];
end

vv_pts = parse_scan(laser_rth,r_pose);

for c_ndx = 1:2:size(vv_pts,2)
    plot(vv_pts(:,c_ndx),vv_pts(:,c_ndx+1), 'r+')
end
%input('pause: classifier 16')

current_hypos = generate_hypotheses(vv_pts);
%current_hypos
r_pose =local1(current_hypos,r_pose);

%input('pause: classifier 22')
%map = zeros(length(laser_rp));