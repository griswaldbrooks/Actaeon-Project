function landmarks = merge_landmarks(prev_land)
%%% When marking records for deletion, set count to zero
landmarks = [];
prev_land
if ~isempty(prev_land)
    for pl_ndx1 = 1:size(prev_land,1)
        error = 1e6;
        sm_land_ndx = 0;
        large_land_ndx = 0;
        %%% Find the closest neighbor %%%
        if (prev_land(pl_ndx1,8)~=0) %%% If the record isn't marked for deletion
            for pl_ndx2 = 1:size(prev_land,1)
                if (prev_land(pl_ndx2,8)~=0) && (pl_ndx1 ~= pl_ndx2)%%% If the record isn't marked for deletion
                    error = sqrt((prev_land(pl_ndx1,5) - prev_land(pl_ndx2,5))^2 + (prev_land(pl_ndx1,6) - prev_land(pl_ndx2,6))^2);
                    %%% Find the longer landmark %%%
                    pl1_len = sqrt((prev_land(pl_ndx1,1) + prev_land(pl_ndx1,3))^2 + (prev_land(pl_ndx1,2) + prev_land(pl_ndx1,4))^2);
                    pl2_len = sqrt((prev_land(pl_ndx2,1) + prev_land(pl_ndx2,3))^2 + (prev_land(pl_ndx2,2) + prev_land(pl_ndx2,4))^2);
                    if pl1_len > pl2_len
                        large_land_ndx = pl_ndx1;
                        sm_land_ndx = pl_ndx2;
                    else
                        large_land_ndx = pl_ndx2;
                        sm_land_ndx = pl_ndx1;
                    end
                end
                
                %%% If your neighbor is within your variance circle or vice versa,
                %%% merge %%%
                if (large_land_ndx~=0) && (error < prev_land(large_land_ndx,7))
                    count = prev_land(large_land_ndx,8) + prev_land(sm_land_ndx,8);
                    %%% Update Mean %%% % Mean is updated via the variance, the
                    % lower the variance, the larger the weight %
                    sum_var = prev_land(large_land_ndx,7) + prev_land(sm_land_ndx,7);
                    prev_land(large_land_ndx,5) = (prev_land(sm_land_ndx,7)/sum_var)*prev_land(large_land_ndx,5) + (prev_land(large_land_ndx,7)/sum_var)*prev_land(sm_land_ndx,5);
                    prev_land(large_land_ndx,6) = (prev_land(sm_land_ndx,7)/sum_var)*prev_land(large_land_ndx,6) + (prev_land(large_land_ndx,7)/sum_var)*prev_land(sm_land_ndx,6);
                    %%% Update Variance %%%
                    prev_land(large_land_ndx,7) = (prev_land(large_land_ndx,8)/count)*prev_land(large_land_ndx,7) + (prev_land(sm_land_ndx,8)/count)*prev_land(sm_land_ndx,7);
                    %%% Update Count %%%
                    prev_land(large_land_ndx,8) = count;
%                     % Update Endpoints %
%                     prev_land(pl_ndx2,1) = (prev_land(pl_ndx1,8)/count)*prev_land(pl_ndx1,1) + (prev_land(pl_ndx2,8)/count)*prev_land(pl_ndx2,1);
%                     prev_land(pl_ndx2,2) = (prev_land(pl_ndx1,8)/count)*prev_land(pl_ndx1,2) + (prev_land(pl_ndx2,8)/count)*prev_land(pl_ndx2,2);
%                     prev_land(pl_ndx2,3) = (prev_land(pl_ndx1,8)/count)*prev_land(pl_ndx1,3) + (prev_land(pl_ndx2,8)/count)*prev_land(pl_ndx2,3);
%                     prev_land(pl_ndx2,4) = (prev_land(pl_ndx1,8)/count)*prev_land(pl_ndx1,4) + (prev_land(pl_ndx2,8)/count)*prev_land(pl_ndx2,4);
%                     dy = prev_land(pl_ndx2,2) - prev_land(pl_ndx2,4);
%                     dx = prev_land(pl_ndx2,1) - prev_land(pl_ndx2,3);
%                     angle = atan2(dy,dx);
%                     r_len = sqrt(dy^2 + dx^2)/2;
%             
%                     an_0 = abs(angle);
%                     an_90 = abs(angle - pi/2);
%                     an_n90 = abs(angle + pi/2);
%                     an_180 = abs(angle - pi);
%                     if an_180 < an_0
%                         an_0 = an_180;
%                     end
%                     if an_n90 < an_90
%                         an_90 = an_n90;
%                     end
%                     if an_0 < an_90 %%% 0 degree orientation
%                         end_pt1 = [prev_land(pl_ndx2,5) - r_len, prev_land(pl_ndx2,6)];
%                         end_pt2 = [prev_land(pl_ndx2,5) + r_len, prev_land(pl_ndx2,6)];
%                     else %%% 90 degree orientation
%                         end_pt1 = [prev_land(pl_ndx2,5), prev_land(pl_ndx2,6) - r_len];
%                         end_pt2 = [prev_land(pl_ndx2,5), prev_land(pl_ndx2,6) + r_len];
%                     end
%                     prev_land(pl_ndx2,1) = end_pt1(1);
%                     prev_land(pl_ndx2,2) = end_pt1(2);
%                     prev_land(pl_ndx2,3) = end_pt2(1);
%                     prev_land(pl_ndx2,4) = end_pt2(2);
                    %%% Mark pl_ndx1 for deletion
                    prev_land(sm_land_ndx,8) = 0;
                    prev_land
                end
            end
        end
    end
    
    %%% Delete obsolete records %%%
    for pl_ndx = size(prev_land,1):-1:1
        pl_ndx
        prev_land
        if prev_land(pl_ndx,8) == 0
            prev_land(pl_ndx,:) = [];
        end
        
    end
    landmarks = prev_land;
    %%% Draw Variances %%%
    for l_ndx = 1:size(landmarks,1)
        draw_circle(landmarks(l_ndx,5),landmarks(l_ndx,6),landmarks(l_ndx,7),'k')
    end
end