function r_pose = local1(laser_xy,r_pose)

angle_increment = 10*(pi/180);

wall1 = [laser_xy(1,1) - laser_xy(35,1), laser_xy(1,2) - laser_xy(35,2)];
wall2 = [laser_xy(10,1) - laser_xy(8,1), laser_xy(10,2) - laser_xy(8,2)];
wall3 = [laser_xy(19,1) - laser_xy(17,1), laser_xy(19,2) - laser_xy(17,2)];
wall4 = [laser_xy(28,1) - laser_xy(26,1), laser_xy(27,2) - laser_xy(26,2)];

ray1 = [laser_xy(1,1) - r_pose(1), laser_xy(1,2) - r_pose(2)];
ray2 = [laser_xy(10,1) - r_pose(1), laser_xy(10,2) - r_pose(2)];
ray3 = [laser_xy(19,1) - r_pose(1), laser_xy(19,2) - r_pose(2)];
ray4 = [laser_xy(28,1) - r_pose(1), laser_xy(28,2) - r_pose(2)];

%cos(th) = (dot(v1,v2))/(abs(v1)*abs(v2));
th1 = acos(dot(wall1,ray1)/(sqrt(wall1(1)^2 + wall1(2)^2)*sqrt(ray1(1)^2 + ray1(2)^2)));
th2 = acos(dot(wall2,ray2)/(sqrt(wall2(1)^2 + wall2(2)^2)*sqrt(ray2(1)^2 + ray2(2)^2)));
th3 = acos(dot(wall3,ray3)/(sqrt(wall3(1)^2 + wall3(2)^2)*sqrt(ray3(1)^2 + ray3(2)^2)));
th4 = acos(dot(wall4,ray4)/(sqrt(wall4(1)^2 + wall4(2)^2)*sqrt(ray4(1)^2 + ray4(2)^2)));

th1 = pi - (th1 + 1*angle_increment);
th2 = pi - (th2 + 10*angle_increment);
th3 = pi - (th3 + 19*angle_increment);
th4 = pi - (th4 + 28*angle_increment);

angles1 = [th1, th1 + pi/2, th1 + pi, th1 - pi/2]
angles2 = [th2, th2 + pi/2, th2 + pi, th2 - pi/2]
angles3 = [th3, th3 + pi/2, th3 + pi, th3 - pi/2]
angles4 = [th4, th4 + pi/2, th4 + pi, th4 - pi/2]

candidate_angles = [angles1,angles2,angles3,angles4]
%%% Find similar angles %%%
c_angles = [];
error = 1*(pi/180);
for c_ndx1 = 1:length(candidate_angles)
    for c_ndx2 = c_ndx1:length(candidate_angles)
        if abs(candidate_angles(c_ndx1) - candidate_angles(c_ndx2)) <= error
            c_angles = [c_angles, mean([candidate_angles(c_ndx1),candidate_angles(c_ndx2)]) ];
        end
    end
end

%%% Find the angle that is closest to the current pose estimate
min_ndx = 0;
min_angle = 3*pi;
for c_ndx = 1:length(c_angles)
    d_angle = c_angles(c_ndx) - r_pose(3);
    if d_angle < min_angle
        min_angle = d_angle;
        min_ndx = c_ndx;
    end
end
r_pose(3)
weight = 0.01*abs(r_pose(3) - c_angles(min_ndx));
r_pose(3) = (1 - weight)*r_pose(3) + (weight)*c_angles(min_ndx)
