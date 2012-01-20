function [r_pose]=correct_trans(r_pose,laser_rp,landmarks)

sig = 1;
angle_increment = 10*(pi/180);
err_xy = [];
for t_iter = 1:20
for laser_ndx = 1:size(laser_rp,2)
    min_land = 0;
    min_dist = 1e6;
    angle = laser_ndx*angle_increment + r_pose(3);
    for land_ndx = 1:size(landmarks,1)
        slope_land = (landmarks(land_ndx,4) - landmarks(land_ndx,2))/(landmarks(land_ndx,3) - landmarks(land_ndx,1));
        slope_laser = tan(angle);
        x = (landmarks(land_ndx,2) - r_pose(2) + slope_laser*r_pose(1) - slope_land*landmarks(land_ndx,1))/(slope_laser - slope_land);
        x_true = ((x < landmarks(land_ndx,3)) && (x > landmarks(land_ndx,1)))||((x > landmarks(land_ndx,3)) && (x < landmarks(land_ndx,1)));
        if x_true
            y = slope_laser*(x - r_pose(1)) + r_pose(2);
            dist = sqrt((x - r_pose(1))^2 + (y - r_pose(2))^2);
            if dist < min_dist
                min_dist = dist;
                min_land = land_ndx;
            end
        end
    end
    d_dist = laser_rp(laser_ndx) - min_dist;
    dx = d_dist*cos(angle);
    dy = d_dist*sin(angle);
    err_xy = [err_xy;dx,dy];
end

err_xy;

x_err = 0;
y_err = 0;
if ~isempty(err_xy)
    m = 4;
    c = 30 - t_iter;
    1;
    err_xy(:,1);
    w_x = 1 - (err_xy(:,1).^m)./((err_xy(:,1).^m) + c^m);
    w_x_err = w_x.*err_xy(:,1);
    sw_x_err = sum(w_x_err);
    sw_x = sum(w_x);
    x_err = sw_x_err/sw_x;
    
    2;
    err_xy(:,2);
    w_y = 1 - (err_xy(:,2).^m)./((err_xy(:,2).^m) + c^m);
    w_y_err = w_y.*err_xy(:,2);
    sw_y_err = sum(w_y_err);
    sw_y = sum(w_y);
    y_err = sw_y_err/sw_y;

end

x_err;
y_err;
%k = (sig^2)/(sig^2 + 500^2);
%k = 0.0001*t_iter;
k = 0.1;
r_pose(1) = r_pose(1) + k*x_err;
r_pose(2) = r_pose(2) + k*y_err;
sig = (1 - k)*sig;
plot(r_pose(1),r_pose(2), 'ko')
%pause(1/64)
err_xy = [];
end