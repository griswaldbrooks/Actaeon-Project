function [var_orth, Line] = isline(pts)

%%% Is line? %%%
    %%% Linear Regression %%%
    pt_avg_x = 0;
    pt_avg_y = 0;
    var_xx = 0;
    var_xy = 0;
    var_yy = 0;
    for ndx = 1:size(pts,1)
      pt_avg_x = (1/ndx)*pts(ndx,1) + ((ndx - 1)/ndx)*pt_avg_x;
      pt_avg_y = (1/ndx)*pts(ndx,2) + ((ndx - 1)/ndx)*pt_avg_y;
    end
    for ndx = 1:size(pts,1)
      var_xx = (1/ndx)*(pt_avg_x - pts(ndx,1))*(pt_avg_x - pts(ndx,1)) + ((ndx - 1)/ndx)*var_xx;
      var_xy = (1/ndx)*(pt_avg_x - pts(ndx,1))*(pt_avg_y - pts(ndx,2)) + ((ndx - 1)/ndx)*var_xy;
      var_yy = (1/ndx)*(pt_avg_y - pts(ndx,2))*(pt_avg_y - pts(ndx,2)) + ((ndx - 1)/ndx)*var_yy;
    end
    if var_xy == 0
         slope = 0; 
    else 
         slope = (var_yy - var_xx + sqrt((var_yy - var_xx)^2 + 4*(var_xy)^2))/(2*var_xy); 
    end
    intercept = pt_avg_y - slope*pt_avg_x;
    
    %%% Calculate Orthogonal Variance %%%
    var_orth = 0;
    for ndx = 1:size(pts,1)
        %%% Calculate x orthogonal projection
        x_orth = pts(ndx,1) + (slope/((slope)^2 + 1))*(pts(ndx,2) - intercept - slope*pts(ndx,1));
        y_orth = slope*x_orth + intercept;
        
        var_orth = (1/ndx)*((x_orth - pts(ndx,1))^2 + (y_orth - pts(ndx,2))^2) + ((ndx-1)/ndx)*var_orth;
    end
    
    Line = [0,0,0,0];
    Line(1) = pts(1,1) + (slope/((slope)^2 + 1))*(pts(1,2) - intercept - slope*pts(1,1));
    Line(2) = slope*Line(1) + intercept;
    Line(3) = pts(size(pts,1),1) + (slope/((slope)^2 + 1))*(pts(size(pts,1),2) - intercept - slope*pts(size(pts,1),1));
    Line(4) = slope*Line(3) + intercept;
 %%% %%%