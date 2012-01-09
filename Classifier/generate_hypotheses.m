function current_hypos = generate_hypotheses(vv_pts)

current_hypos = [];
for pts_ndx = 1:2:size(vv_pts,2)

    temp_ftrs = produce_feature(vv_pts(:,pts_ndx:(pts_ndx+1)));
    current_hypos = [current_hypos;temp_ftrs];
    current_hypos(all(current_hypos==0,2),:) = [];

end
%current_hypos