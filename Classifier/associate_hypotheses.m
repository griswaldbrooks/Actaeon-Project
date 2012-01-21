function [landmarks] = associate_hypotheses(prev_land, current_hypos)
%%% landmark structure = [endpoint x1, endpoint y1,
%%%                       endpoint x2, endpoint y2,
%%%                       center x, center y,
%%%                       variance, count]
landmarks = [];
x_y_thresh = 15; %cm
if isempty(prev_land)
    for ch_ndx = 1:size(current_hypos,1)
        dy = current_hypos(ch_ndx,2) - current_hypos(ch_ndx,4);
        dx = current_hypos(ch_ndx,1) - current_hypos(ch_ndx,3);
        angle = atan2(dy,dx);
        r_len = sqrt(dy^2 + dx^2)/2;
        center = [mean([current_hypos(ch_ndx,1),current_hypos(ch_ndx,3)]),mean([current_hypos(ch_ndx,2),current_hypos(ch_ndx,4)])];
        an_0 = abs(angle);
        an_90 = abs(abs(angle) - pi/2);
        an_n90 = abs(abs(angle) + pi/2);
        an_180 = abs(abs(angle) - pi);
        if an_180 < an_0
            an_0 = an_180;
        end
        if an_n90 < an_90
            an_90 = an_n90;
        end
        if an_0 < an_90 %%% 0 degree orientation
            end_pt1 = [center(1) - r_len, center(2)];
            end_pt2 = [center(1) + r_len, center(2)];
        else %%% 90 degree orientation
            end_pt1 = [center(1), center(2) - r_len];
            end_pt2 = [center(1), center(2) + r_len];
        end
        new_land = [end_pt1,end_pt2,center,r_len/2,1];
        landmarks = [landmarks;new_land];
    end
else
    for ch_ndx = 1:size(current_hypos,1)
        min_pl_ndx = 0;
        min_dist = 1e6;
        for pl_ndx = 1:size(prev_land,1)
            center = [mean([current_hypos(ch_ndx,1),current_hypos(ch_ndx,3)]),mean([current_hypos(ch_ndx,2),current_hypos(ch_ndx,4)])];
            error = sqrt((prev_land(pl_ndx,5) - center(1))^2 + (prev_land(pl_ndx,6) - center(2))^2);
            if error < min_dist
                min_dist = error;
                min_pl_ndx = pl_ndx;
            end
        end
        prev_land(min_pl_ndx,7)
        if (min_dist) < (prev_land(min_pl_ndx,7)) %%% If the distance between centers is within the variance circle
            center_h = [mean([current_hypos(ch_ndx,1),current_hypos(ch_ndx,3)]),mean([current_hypos(ch_ndx,2),current_hypos(ch_ndx,4)])];
            count = prev_land(min_pl_ndx,8);
            % Update Variance %
            variance = (1/count)*(min_dist) + ((count - 1)/count)*prev_land(min_pl_ndx,7);
            prev_land(min_pl_ndx,7) = variance;
            % Update Count %
            prev_land(min_pl_ndx,8) = prev_land(min_pl_ndx,8) + 1;

        elseif  overlap(prev_land(min_pl_ndx,:),current_hypos(ch_ndx,:)) && close_x_or_y(prev_land(min_pl_ndx,:),current_hypos(ch_ndx,:),x_y_thresh)%%% End points overlap and sufficiently close
            count = prev_land(min_pl_ndx,8);
            % Update Variance %
            variance = (1/count)*(min_dist) + ((count - 1)/count)*prev_land(min_pl_ndx,7);
            prev_land(min_pl_ndx,7) = variance;
            % Update Count %
            prev_land(min_pl_ndx,8) = prev_land(min_pl_ndx,8) + 1;
        else
            dy = current_hypos(ch_ndx,2) - current_hypos(ch_ndx,4);
            dx = current_hypos(ch_ndx,1) - current_hypos(ch_ndx,3);
            angle = atan2(dy,dx);
            r_len = sqrt(dy^2 + dx^2)/2;
            if r_len > 0.001 %%% Ignore short hypotheses
                center = [mean([current_hypos(ch_ndx,1),current_hypos(ch_ndx,3)]),mean([current_hypos(ch_ndx,2),current_hypos(ch_ndx,4)])];            
                an_0 = abs(angle);
                an_90 = abs(abs(angle) - pi/2);
                an_n90 = abs(abs(angle) + pi/2);
                an_180 = abs(abs(angle) - pi);
                if an_180 < an_0
                    an_0 = an_180;
                end
                if an_n90 < an_90
                    an_90 = an_n90;
                end
                if an_0 < an_90 %%% 0 degree orientation
                    end_pt1 = [center(1) - r_len, center(2)];
                    end_pt2 = [center(1) + r_len, center(2)];
                else %%% 90 degree orientation
                    end_pt1 = [center(1), center(2) - r_len];
                    end_pt2 = [center(1), center(2) + r_len];
                end
                new_land = [end_pt1,end_pt2,center,r_len/2,1];            
                landmarks = [landmarks;new_land];
            end
        end
    end
    landmarks = [prev_land;landmarks];
end

%%% Draw Variances %%%
for l_ndx = 1:size(landmarks,1)
    draw_circle(landmarks(l_ndx,5),landmarks(l_ndx,6),landmarks(l_ndx,7),'r')
end
%input('Pause: associate_hypo 124')
