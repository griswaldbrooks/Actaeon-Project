function [r_pose]=correct_trans(r_pose,laser_rp,map)

scaler = 8;
sig = 1;
for t_iter = 1:20
occ_hit = [];
err_xy = [];
r_iter = 0;
map_max = length(map);
map_min = 0;
angle_increment = 10*(pi/180);
for l_ndx = 1:length(laser_rp)
    angle = l_ndx*angle_increment + r_pose(3);
    x_iter = round(r_iter*cos(angle) + r_pose(1)/scaler) + 15;
    y_iter = round(r_iter*sin(angle) + r_pose(2)/scaler) + 15;
    while (isempty(occ_hit)) && (x_iter < map_max) && (y_iter < map_max) && (x_iter > map_min) && (y_iter > map_min)
        if map(x_iter,y_iter) > 5
            x_iter = (x_iter - 15)*scaler;
            y_iter = (y_iter - 15)*scaler;
            occ_hit = [x_iter,y_iter];
        elseif map(x_iter,y_iter) == 0
            break;
        end
        r_iter = r_iter + 1;
        x_iter = round(r_iter*cos(angle) + r_pose(1)/scaler) + 15;
        y_iter = round(r_iter*sin(angle) + r_pose(2)/scaler) + 15;
    end
    x_l = laser_rp(l_ndx)*cos(angle);
    y_l = laser_rp(l_ndx)*sin(angle);
    if ~isempty(occ_hit)
        error = [x_l - occ_hit(1),y_l - occ_hit(2)];
        err_xy = [err_xy; error];
%        plot((occ_hit(1)-15)*scaler,(occ_hit(2)-15)*scaler,'ks')
        plot(occ_hit(1),occ_hit(2),'ks-')
    end
    
    occ_hit = [];
end

err_xy;
x_err = 0;
y_err = 0;
if ~isempty(err_xy)
    m = 3;
    c = 30 - t_iter;
    1
    err_xy(:,1)
    w_x = 1 - (err_xy(:,1).^m)./((err_xy(:,1).^m) + c^m)
    w_x_err = w_x.*err_xy(:,1);
    sw_x_err = sum(w_x_err);
    sw_x = sum(w_x);
    x_err = sw_x_err/sw_x
    
    2
    err_xy(:,2)
    w_y = 1 - (err_xy(:,2).^m)./((err_xy(:,2).^m) + c^m)
    w_y_err = w_y.*err_xy(:,2);
    sw_y_err = sum(w_y_err);
    sw_y = sum(w_y);
    y_err = sw_y_err/sw_y

end

x_err
y_err
k = (sig^2)/(sig^2 + 500^2);
r_pose(1) = r_pose(1) + k*x_err;
r_pose(2) = r_pose(2) + k*y_err;
sig = (1 - k)*sig;
plot(r_pose(1),r_pose(2), 'ko')
%pause(1/64)
end