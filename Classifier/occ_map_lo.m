%%% Mapping only, sets the points in the 3x3 matrix centered at a distance
%%% measurement to the cumulative multiplicative mean.
%%% This means that the current value of the elements around a measurements are multiplied by
%%% the mean of the 3x3 surrounding elements. They are them summed with
%%% their previous value. The measurement location is then incremented by
%%% one.

function map_p = slam_mean(laser_rp,r_pose,map)
scaler = 8;
map_dim = length(map);
window_dim = 1;
angle_increment = 10*(pi/180);
laser_xy = zeros(length(laser_rp),2);
%map_p = zeros(length(map));
%map_p = map;

%%% Calculate occlusion coordinates %%%
for index = 1:length(laser_rp)
    angle = index*angle_increment + r_pose(3);
    x = round( (laser_rp(index)*cos(angle) + r_pose(1))/scaler );
    y = round( (laser_rp(index)*sin(angle) + r_pose(2))/scaler );
    x = x + 15;
    y = y + 15;
    laser_xy(index,:) = [x,y];
end

%%% Update occlusions probabilistically %%%
for occ_ndx = 1:length(laser_xy)
    %%% Check window bounds %%%    
    offset = floor(window_dim/2);
    if (laser_xy(occ_ndx,1) - offset) < 1
         row_min = 1;
    else
         row_min = laser_xy(occ_ndx,1) - offset;
    end
    if (laser_xy(occ_ndx,1) + offset) >= map_dim
         row_max = map_dim;
    else
         row_max = laser_xy(occ_ndx,1) + offset;
    end
    if (laser_xy(occ_ndx,2) - offset) < 1
         col_min = 1;
    else
         col_min = laser_xy(occ_ndx,2) - offset;
    end
    if (laser_xy(occ_ndx,2) + offset) >= map_dim
         col_max = map_dim;
    else
         col_max = laser_xy(occ_ndx,2) + offset;
    end
    
     if laser_xy(occ_ndx,1) < 1
            laser_xy(occ_ndx,1) = 1;
        elseif laser_xy(occ_ndx,1) > length(map)
            laser_xy(occ_ndx,1) = length(map);
        end
        if laser_xy(occ_ndx,2) < 1
            laser_xy(occ_ndx,2) = 1;
        elseif laser_xy(occ_ndx,2) > length(map)
            laser_xy(occ_ndx,2) = length(map);
        end
    
    
    %%% Update free space elements %%%
    angle = occ_ndx*angle_increment + r_pose(3);
    prb_free = 0.35;
    l_free = log(prb_free/(1 - prb_free));
    x_iter = round(r_pose(1)/scaler);
    y_iter = round(r_pose(2)/scaler);
    r_iter = 0;
    free_xy = [];
    
    while r_iter < (laser_rp(occ_ndx)/scaler)
        if ((y_iter < col_min) || (y_iter > col_max) && (x_iter < row_min) || (x_iter > row_max))
            x_iter = round(r_iter*cos(angle) + r_pose(1)/scaler) + 15;
            y_iter = round(r_iter*sin(angle) + r_pose(2)/scaler) + 15;
            free_xy = [free_xy;x_iter,y_iter];
        end
        r_iter = r_iter + 1;
            
    end
    
        
    free_xy = unique(free_xy,'rows');
    if ~(isempty(free_xy))
        for free_ndx = 1:length(free_xy(:,1))
            x = free_xy(free_ndx,1);
            y = free_xy(free_ndx,2);
            
            if x < 1
                x = 1;
            elseif x > length(map)
                x = length(map);
            end
            if y < 1
                y = 1;
            elseif y > length(map)
                y = length(map);
            end
        
            map(x,y) = map(x,y) + l_free;
%             if map(x, y) < 0
%                 map(x, y) = 0;
%             elseif map(x,y) > 1
%                 map(x,y) = 1;
%             end
        end
    end
    free_xy = [];
    
    %%% Update occlusion elements %%%
    for r_ndx = row_min:row_max
         for c_ndx = col_min:col_max        
              sigma = 1;
              mu = sqrt(laser_xy(occ_ndx,1)^2 + laser_xy(occ_ndx,2)^2);
              x = sqrt(r_ndx^2 + c_ndx^2);
              prb = normpdf(x,mu,sigma) + 0.4;
              l_occ = log(prb/(1-prb));
              map(r_ndx, c_ndx) = map(r_ndx, c_ndx) + l_occ;
%               if map(r_ndx, c_ndx) < 0
%                   map(r_ndx, c_ndx) = 0;
%               elseif map(r_ndx, c_ndx) > 1
%                   map(r_ndx,c_ndx) = 1;
%               end
         end
    end
    
end

map_p = map;

