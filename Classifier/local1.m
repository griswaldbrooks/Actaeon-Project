function [r_pose,xn,yn] = local1(walls_h,r_pose,xn,yn,laser_rp,landmarks)

%%% Correct for Orientation Error %%%
[r_pose,xn,yn] = correct_orient(walls_h,r_pose,xn,yn);

%%% Correct for Translation Error %%%
[r_pose]=correct_trans(r_pose,laser_rp,landmarks);










