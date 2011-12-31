function clusters = k_lines(k,pts)
%%% Cluster structure is
 %%% [c1x1,c1y1,c2x1,c2y1,...,cnx1,cny1;c1x2,c1y2,...,cnx2,cny2;...;c1xm,c1
 %%% ym,...,cnxm,cnym]
pt_cl = [pts,zeros(size(pts,1),1)]; % Copy points and give them a default association of k, since the value of zero will never be a cluster assignment
%clusters = zeros(size(pt_cl,1),k);
%%% Generate initial clusters
clusters = k_means(k,pts);
new_lines = zeros(k,2);
old_lines = zeros(k,2);
distances = zeros(1,k);

%clusters,13
%input('pause: k_lines 14')

%%% Generate initial lines, reset clusters
for c_ndx = 1:2:(2*k-1)
    x_vect = clusters(:,c_ndx);
    y_vect = clusters(:,c_ndx+1);
    %%% Trim Zeros %%%
    x_vect(all(x_vect==0,2),:) = [];
    y_vect(all(y_vect==0,2),:) = [];
%%% Linear Regression %%%
    pt_avg_x = 0;
    pt_avg_y = 0;
    var_xx = 0;
    var_xy = 0;
    var_yy = 0;
    for ndx = 1:size(x_vect,1)
      pt_avg_x = (1/ndx)*x_vect(ndx) + ((ndx - 1)/ndx)*pt_avg_x;
      pt_avg_y = (1/ndx)*y_vect(ndx) + ((ndx - 1)/ndx)*pt_avg_y;
    end
    for ndx = 1:size(x_vect,1)
      var_xx = (1/ndx)*(pt_avg_x - x_vect(ndx))*(pt_avg_x - x_vect(ndx)) + ((ndx - 1)/ndx)*var_xx;
      var_xy = (1/ndx)*(pt_avg_x - x_vect(ndx))*(pt_avg_y - y_vect(ndx)) + ((ndx - 1)/ndx)*var_xy;
      var_yy = (1/ndx)*(pt_avg_y - y_vect(ndx))*(pt_avg_y - y_vect(ndx)) + ((ndx - 1)/ndx)*var_yy;
    end
    if var_xy == 0
         slope = 0; 
    else 
         slope = (var_yy - var_xx + sqrt((var_yy - var_xx)^2 + 4*(var_xy)^2))/(2*var_xy); 
    end
    intercept = pt_avg_y - slope*pt_avg_x;
%%%
    %clusters,45
    %input('pause: k_lines 46')
    
    new_lines((c_ndx+1)/2,:) = [slope,intercept];
    clusters(:,c_ndx:(c_ndx+1)) = zeros(size(clusters,1),2);
end

%%% Start Main Loop
while(any(any(old_lines ~= new_lines)))
    %%% Reassign Points
    for pt_ndx = 1:size(pt_cl,1) 
        %%% Determine the distances from each point to each line
        for k_ndx = 1:k
            slope = new_lines(k_ndx,1);
            intercept = new_lines(k_ndx,2);
            x_orth = pt_cl(pt_ndx,1) + (slope/((slope)^2 + 1))*(pt_cl(pt_ndx,2) - intercept - slope*pt_cl(pt_ndx,1));
            y_orth = slope*x_orth + intercept;
            distances(k_ndx) = sqrt((x_orth - pt_cl(pt_ndx,1))^2 + (y_orth - pt_cl(pt_ndx,2))^2);
        end
        %%% Arg min %%%
        min_dis = 100000000000000;
        for k_ndx = 1:k
            if distances(k_ndx) < min_dis
                min_k = k_ndx;
                min_dis = distances(k_ndx);
            end
        end
        pt_cl(pt_ndx,3) = min_k;
    end
    %%% Clear Clusters
    clusters(:,c_ndx:(c_ndx+1)) = zeros(size(clusters,1),2);
    %%% Generate Clusters
    for pt_ndx = 1:size(pt_cl,1)
        cl_ndx = 2*pt_cl(pt_ndx,3) - 1;
        clusters(pt_ndx,cl_ndx:(cl_ndx+1)) = pt_cl(pt_ndx,1:2);
    end
    %clusters,81
    %input('pause: k_lines 76')
    %%% Calculate New Lines
    old_lines = new_lines;
    new_lines = zeros(k,2);
    
    
    %--------------------------
    for c_ndx = 1:2:(2*k-1)
        x_vect = clusters(:,c_ndx);
        y_vect = clusters(:,c_ndx+1);
        %%% Trim Zeros %%%
        x_vect(all(x_vect==0,2),:) = [];
        y_vect(all(y_vect==0,2),:) = [];

        %%% Linear Regression %%%
        pt_avg_x = 0;
        pt_avg_y = 0;
        var_xx = 0;
        var_xy = 0;
        var_yy = 0;
        for ndx = 1:size(x_vect,1)
            pt_avg_x = (1/ndx)*x_vect(ndx) + ((ndx - 1)/ndx)*pt_avg_x;
            pt_avg_y = (1/ndx)*y_vect(ndx) + ((ndx - 1)/ndx)*pt_avg_y;
        end
        for ndx = 1:size(x_vect,1)
            var_xx = (1/ndx)*(pt_avg_x - x_vect(ndx))*(pt_avg_x - x_vect(ndx)) + ((ndx - 1)/ndx)*var_xx;
            var_xy = (1/ndx)*(pt_avg_x - x_vect(ndx))*(pt_avg_y - y_vect(ndx)) + ((ndx - 1)/ndx)*var_xy;
            var_yy = (1/ndx)*(pt_avg_y - y_vect(ndx))*(pt_avg_y - y_vect(ndx)) + ((ndx - 1)/ndx)*var_yy;
        end
        if var_xy == 0
            slope = 0; 
        else 
            slope = (var_yy - var_xx + sqrt((var_yy - var_xx)^2 + 4*(var_xy)^2))/(2*var_xy); 
        end
        intercept = pt_avg_y - slope*pt_avg_x;
        %%%

        new_lines((c_ndx+1)/2,:) = [slope,intercept];
        clusters(:,c_ndx:(c_ndx+1)) = zeros(size(clusters,1),2);
    end
    %-------------------------- 
end

%%% Check neighbor assignments for cluster assignment corrections
%%% Find cluster assignment of neighbors
for pt_ndx = 2:(size(pt_cl,1)-1)
    if (pt_cl(pt_ndx,3) == pt_cl(pt_ndx-1,3)) && (pt_cl(pt_ndx,3) == pt_cl(pt_ndx+1,3))
        %%% No reassignment necessary
    elseif (pt_cl(pt_ndx+1,3) == pt_cl(pt_ndx-1,3))
        %%% If its neighbors are in the same cluster, switch assignment
        pt_cl(pt_ndx,3) = pt_cl(pt_ndx-1,3);
    end
end
    
%%% Reassign the clusters
clusters = zeros(size(pt_cl,1),k);
for pt_ndx = 1:size(pt_cl,1)
    cl_ndx = 2*pt_cl(pt_ndx,3) - 1;
    clusters(pt_ndx,cl_ndx:(cl_ndx+1)) = pt_cl(pt_ndx,1:2);
end