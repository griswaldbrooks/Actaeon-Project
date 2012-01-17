function [landmarks] = associate_hypotheses(prev_land, current_hypos)
%%% landmark structure = [endpoint x1, endpoint y1,
%%%                       endpoint x2, endpoint y2,
%%%                       center x, center y,
%%%                       variance, count]
landmarks = [];
gate = 1;
if isempty(prev_land)
    for ch_ndx = 1:size(current_hypos,1)
        center = [mean([current_hypos(ch_ndx,1),current_hypos(ch_ndx,3)]),mean([current_hypos(ch_ndx,2),current_hypos(ch_ndx,4)])];
        new_land = [current_hypos(ch_ndx,:),center,0,1];
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
        if gate > (min_dist^2)*(prev_land(min_pl_ndx,7))
            center_h = [mean([current_hypos(ch_ndx,1),current_hypos(ch_ndx,3)]),mean([current_hypos(ch_ndx,2),current_hypos(ch_ndx,4)])];
            center_lnd = [mean([center_h(1),prev_land(min_pl_ndx,5)]),mean([center_h(2),prev_land(min_pl_ndx,6)])];
            
            count = count + 1;
        end
    end
end
