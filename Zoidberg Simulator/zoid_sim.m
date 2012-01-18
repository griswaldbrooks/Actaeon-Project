%%% Flame Simulation %%%
cla, clc, clear
hold all
axis equal

flame_pose = [0,15]; %cm
axis([-15,15,-10,flame_pose(2) + 10])
zoid_off_t = 6.3;
zoid_off_s15 = 1.8;
zoid_off_s24 = 0.8;
view_angle = 20*(pi/180);
blinder_length = 1.8;
blinder_off = 0.35;
flame_max = 150;
zoid_array = [flame_max,flame_max,flame_max,flame_max,flame_max];
dt = 0.1;
for t = 0:dt:30
cla
%flame_pose = [-15+t,15];
%%% Plot Flame %%%
plot(flame_pose(1), flame_pose(2), 'rd-')
%%%
r_pose = [0,0,pi/2 + t];
%r_pose = [0,0,pi/2 + 45*(pi/180)];
%r_pose = [0,0,pi/2];
%%% Restrict the Orientation [0,2*pi) %%%
if r_pose(3) >= (2*pi)
    r_pose(3) = r_pose(3) - floor(r_pose(3)/(2*pi))*2*pi;
end
%%% Plot Robot Center %%%
plot(r_pose(1),r_pose(2), 'ks')
%%%
zoid_pose = [r_pose(1) + zoid_off_t*cos(r_pose(3)),r_pose(2) + zoid_off_t*sin(r_pose(3)), r_pose(3)];
zoid_s1 = [zoid_pose(1) + zoid_off_s15*cos(zoid_pose(3) + pi/2),zoid_pose(2) + zoid_off_s15*sin(zoid_pose(3) + pi/2),zoid_pose(3)];
zoid_s2 = [zoid_pose(1) + zoid_off_s24*cos(zoid_pose(3) + pi/2),zoid_pose(2) + zoid_off_s24*sin(zoid_pose(3) + pi/2),zoid_pose(3)];
zoid_s3 = [zoid_pose(1),zoid_pose(2),zoid_pose(3)];
zoid_s4 = [zoid_pose(1) + zoid_off_s24*cos(zoid_pose(3) - pi/2),zoid_pose(2) + zoid_off_s24*sin(zoid_pose(3) - pi/2),zoid_pose(3)];
zoid_s5 = [zoid_pose(1) + zoid_off_s15*cos(zoid_pose(3) - pi/2),zoid_pose(2) + zoid_off_s15*sin(zoid_pose(3) - pi/2),zoid_pose(3)];


%%% Plot Zoidberg %%%
plot(zoid_pose(1),zoid_pose(2), 'bs')
plot(zoid_s1(1),zoid_s1(2), 'bo')
plot(zoid_s2(1),zoid_s2(2), 'bo')
plot(zoid_s3(1),zoid_s3(2), 'bo')
plot(zoid_s4(1),zoid_s4(2), 'bo')
plot(zoid_s5(1),zoid_s5(2), 'bo')
%%%
% %%% Plot Zoidberg Arrows %%%
% line([zoid_s1(1),zoid_s1(1) + cos(zoid_s1(3))],[zoid_s1(2),zoid_s1(2) + sin(zoid_s1(3))], 'Color','r')
% line([zoid_s2(1),zoid_s2(1) + cos(zoid_s2(3))],[zoid_s2(2),zoid_s2(2) + sin(zoid_s2(3))], 'Color','r')
% line([zoid_s3(1),zoid_s3(1) + cos(zoid_s3(3))],[zoid_s3(2),zoid_s3(2) + sin(zoid_s3(3))], 'Color','r')
% line([zoid_s4(1),zoid_s4(1) + cos(zoid_s4(3))],[zoid_s4(2),zoid_s4(2) + sin(zoid_s4(3))], 'Color','r')
% line([zoid_s5(1),zoid_s5(1) + cos(zoid_s5(3))],[zoid_s5(2),zoid_s5(2) + sin(zoid_s5(3))], 'Color','r')
% %%%

blinder1 = [zoid_pose(1)+ 3*blinder_off*cos(zoid_pose(3) + pi/2),zoid_pose(2)+ 3*blinder_off*sin(zoid_pose(3) + pi/2), zoid_pose(1)+ 3*blinder_off*cos(zoid_pose(3) + pi/2) + blinder_length*cos(zoid_pose(3)), zoid_pose(2)+ 3*blinder_off*sin(zoid_pose(3) + pi/2) + blinder_length*sin(zoid_pose(3))];
blinder2 = [zoid_pose(1)+ 1*blinder_off*cos(zoid_pose(3) + pi/2),zoid_pose(2)+ 1*blinder_off*sin(zoid_pose(3) + pi/2), zoid_pose(1)+ 1*blinder_off*cos(zoid_pose(3) + pi/2) + blinder_length*cos(zoid_pose(3)), zoid_pose(2)+ 1*blinder_off*sin(zoid_pose(3) + pi/2) + blinder_length*sin(zoid_pose(3))];
blinder3 = [zoid_pose(1)+ 1*blinder_off*cos(zoid_pose(3) - pi/2),zoid_pose(2)+ 1*blinder_off*sin(zoid_pose(3) - pi/2), zoid_pose(1)+ 1*blinder_off*cos(zoid_pose(3) - pi/2) + blinder_length*cos(zoid_pose(3)), zoid_pose(2)+ 1*blinder_off*sin(zoid_pose(3) - pi/2) + blinder_length*sin(zoid_pose(3))];
blinder4 = [zoid_pose(1)+ 3*blinder_off*cos(zoid_pose(3) - pi/2),zoid_pose(2)+ 3*blinder_off*sin(zoid_pose(3) - pi/2), zoid_pose(1)+ 3*blinder_off*cos(zoid_pose(3) - pi/2) + blinder_length*cos(zoid_pose(3)), zoid_pose(2)+ 3*blinder_off*sin(zoid_pose(3) - pi/2) + blinder_length*sin(zoid_pose(3))];
%%% Plot Blinders %%%
line([blinder1(1),blinder1(3)],[blinder1(2),blinder1(4)], 'Color','k')
line([blinder2(1),blinder2(3)],[blinder2(2),blinder2(4)], 'Color','k')
line([blinder3(1),blinder3(3)],[blinder3(2),blinder3(4)], 'Color','k')
line([blinder4(1),blinder4(3)],[blinder4(2),blinder4(4)], 'Color','k')
%%%

%%% Plot Sight Lines %%%
if abs(zoid_pose(3) - atan2(flame_pose(2)- zoid_s1(2),flame_pose(1) - zoid_s1(1))) < view_angle
    slope_f = (flame_pose(2)- zoid_s1(2))/(flame_pose(1) - zoid_s1(1));
    slope_b = (blinder1(2) - blinder1(4))/(blinder1(1) - blinder1(3));
    slopes_eq = (slope_f == slope_b);
    if ~slopes_eq
        x = (blinder1(2) - zoid_s1(2) + slope_f*zoid_s1(1) - slope_b*blinder1(1))/(slope_f - slope_b);
        y = slope_f*(x - zoid_s1(1)) + zoid_s1(2);
        x_true = ((x < blinder1(3)) && (x > blinder1(1)))||((x > blinder1(3)) && (x < blinder1(1)));
        intersect = (x_true);
    end
    if slopes_eq || ~intersect
        line([zoid_s1(1),flame_pose(1)],[zoid_s1(2),flame_pose(2)],'Color','r')
        zoid_array(1) = sqrt((flame_pose(2)- zoid_s1(2))^2 + (flame_pose(1) - zoid_s1(1))^2);
    end
end
if abs(zoid_pose(3) - atan2(flame_pose(2)- zoid_s2(2),flame_pose(1) - zoid_s2(1))) < view_angle
    slope_f = (flame_pose(2)- zoid_s2(2))/(flame_pose(1) - zoid_s2(1));
    slope_b1 = (blinder1(2) - blinder1(4))/(blinder1(1) - blinder1(3));
    slope_b2 = (blinder2(2) - blinder2(4))/(blinder2(1) - blinder2(3));
    slopes_eq = (slope_f == slope_b1) || (slope_f == slope_b2);
    if ~slopes_eq
        x1 = (blinder1(2) - zoid_s2(2) + slope_f*zoid_s2(1) - slope_b1*blinder1(1))/(slope_f - slope_b1);
        x2 = (blinder2(2) - zoid_s2(2) + slope_f*zoid_s2(1) - slope_b2*blinder2(1))/(slope_f - slope_b2);
        x1_true = ((x1 < blinder1(3)) && (x1 > blinder1(1)))||((x1 > blinder1(3)) && (x1 < blinder1(1)));
        x2_true = ((x2 < blinder2(3)) && (x2 > blinder2(1)))||((x2 > blinder2(3)) && (x2 < blinder2(1)));
        intersect = x1_true || x2_true;
    end
    if slopes_eq || ~intersect
        line([zoid_s2(1),flame_pose(1)],[zoid_s2(2),flame_pose(2)],'Color','r')
        zoid_array(2) = sqrt((flame_pose(2)- zoid_s2(2))^2 + (flame_pose(1) - zoid_s2(1))^2);
    end
end
if abs(zoid_pose(3) - atan2(flame_pose(2)- zoid_s3(2),flame_pose(1) - zoid_s3(1))) < view_angle
    slope_f = (flame_pose(2)- zoid_s3(2))/(flame_pose(1) - zoid_s3(1));
    slope_b2 = (blinder2(2) - blinder2(4))/(blinder2(1) - blinder2(3));
    slope_b3 = (blinder3(2) - blinder3(4))/(blinder3(1) - blinder3(3));
    slopes_eq = (slope_f == slope_b2) || (slope_f == slope_b3);
    if ~slopes_eq
        x2 = (blinder2(2) - zoid_s3(2) + slope_f*zoid_s3(1) - slope_b2*blinder2(1))/(slope_f - slope_b2);
        x3 = (blinder3(2) - zoid_s3(2) + slope_f*zoid_s3(1) - slope_b3*blinder3(1))/(slope_f - slope_b3);
        x2_true = ((x2 < blinder2(3)) && (x2 > blinder2(1)))||((x2 > blinder2(3)) && (x2 < blinder2(1)));
        x3_true = ((x3 < blinder3(3)) && (x3 > blinder3(1)))||((x3 > blinder3(3)) && (x3 < blinder3(1)));
        intersect = x2_true || x3_true;
    end
    if slopes_eq || ~intersect
        line([zoid_s3(1),flame_pose(1)],[zoid_s3(2),flame_pose(2)],'Color','r')
        zoid_array(3) = sqrt((flame_pose(2)- zoid_s3(2))^2 + (flame_pose(1) - zoid_s3(1))^2);
    end
end
if abs(zoid_pose(3) - atan2(flame_pose(2)- zoid_s4(2),flame_pose(1) - zoid_s4(1))) < view_angle
    slope_f = (flame_pose(2)- zoid_s4(2))/(flame_pose(1) - zoid_s4(1));
    slope_b3 = (blinder3(2) - blinder3(4))/(blinder3(1) - blinder3(3));
    slope_b4 = (blinder4(2) - blinder4(4))/(blinder4(1) - blinder4(3));
    slopes_eq = (slope_f == slope_b3) || (slope_f == slope_b4);
    if ~slopes_eq
        x3 = (blinder3(2) - zoid_s4(2) + slope_f*zoid_s4(1) - slope_b3*blinder3(1))/(slope_f - slope_b3);
        x4 = (blinder4(2) - zoid_s4(2) + slope_f*zoid_s4(1) - slope_b4*blinder4(1))/(slope_f - slope_b4);
        x3_true = ((x3 < blinder3(3)) && (x3 > blinder3(1)))||((x3 > blinder3(3)) && (x3 < blinder3(1)));
        x4_true = ((x4 < blinder4(3)) && (x4 > blinder4(1)))||((x4 > blinder4(3)) && (x4 < blinder4(1)));
        intersect = x3_true || x4_true;
    end
    if slopes_eq || ~intersect
        line([zoid_s4(1),flame_pose(1)],[zoid_s4(2),flame_pose(2)],'Color','r')
        zoid_array(4) = sqrt((flame_pose(2)- zoid_s4(2))^2 + (flame_pose(1) - zoid_s4(1))^2);
    end
end
if abs(zoid_pose(3) - atan2(flame_pose(2)- zoid_s5(2),flame_pose(1) - zoid_s5(1))) < view_angle
    slope_f = (flame_pose(2)- zoid_s5(2))/(flame_pose(1) - zoid_s5(1));
    slope_b = (blinder4(2) - blinder4(4))/(blinder4(1) - blinder4(3));
    slopes_eq = (slope_f == slope_b);
    if ~slopes_eq
        x = (blinder4(2) - zoid_s5(2) + slope_f*zoid_s5(1) - slope_b*blinder4(1))/(slope_f - slope_b);
        y = slope_f*(x - zoid_s5(1)) + zoid_s5(2);
        x_true = ((x < blinder4(3)) && (x > blinder4(1)))||((x > blinder4(3)) && (x < blinder4(1)));
        intersect = (x_true);
    end
    if slopes_eq || ~intersect
        line([zoid_s5(1),flame_pose(1)],[zoid_s5(2),flame_pose(2)],'Color','r')
        zoid_array(5) = sqrt((flame_pose(2)- zoid_s5(2))^2 + (flame_pose(1) - zoid_s5(1))^2);
    end
end
zoid_array
wv = [15.9*(1/zoid_array(1)),7.24*(1/zoid_array(2)),0,-7.24*(1/zoid_array(4)),-15.9*(1/zoid_array(5))];
s_wv = sum(wv);
s_w = sum([(1/zoid_array(1)),(1/zoid_array(2)),(1/zoid_array(3)),(1/zoid_array(4)),(1/zoid_array(5))]);
angle = s_wv/s_w
pause(dt)
end
