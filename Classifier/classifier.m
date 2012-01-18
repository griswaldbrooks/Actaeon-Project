function [r_pose,xn,yn,landmarks] = classifier(laser_rp,r_pose,xn,yn,map,landmarks)

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
current_hypos
%%% DRAW HYPOTHESES %%%
for iter = 1:2:size(current_hypos,1)
    line([current_hypos(iter,1),current_hypos(iter,3)],[current_hypos(iter,2),current_hypos(iter,4)],'Color','r')
end

[r_pose,xn,yn] = correct_orient(current_hypos,r_pose,xn,yn);

landmarks = associate_hypotheses(landmarks, current_hypos);

%%% DRAW LANDMARKS %%%
for iter = 1:2:size(landmarks,1)
    line([landmarks(iter,1),landmarks(iter,3)],[landmarks(iter,2),landmarks(iter,4)],'Color','g')
end
%input('Pause')
[r_pose,xn,yn] =local1(current_hypos,r_pose,xn,yn,laser_rp,landmarks);

%input('pause: classifier 22')
%map = zeros(length(laser_rp));