function [r_pose,xn,yn] = local1(walls_h,r_pose,xn,yn)
%b =
%[1,(2-exp(-i*(pi/2))-exp(i*(pi/2))),2*(1-exp(-i*(pi/2))-exp(i*(pi/2))),(2-exp(-i*(pi/2))-exp(i*(pi/2))),1]
%0.3468    1.3873    2.0809    1.3873    0.3468
b4 = 0.3468;
b3 = 1.3873;
b2 = 2.0809;
b1 = 1.3873;
b0 = 0.3468;
%1.0000    1.9684    1.7359    0.7245    0.1204
a4 = 1;
a3 = 1.9684;
a2 = 1.7359;
a1 = 0.7245;
a0 = 0.1204;

k = 1.0000e-004;

angle_increment = 10*(pi/180);

%%% Find orientation of the walls
orient_cand = [];
%input('pause: local1 7')
for w_ndx = 1:size(walls_h,1)
    dx = walls_h(w_ndx,1) - walls_h(w_ndx,3);
    dy = walls_h(w_ndx,2) - walls_h(w_ndx,4);
    orient_cand = [orient_cand,atan2(dy,dx)];
end

orient_hypos = [orient_cand,orient_cand+(pi/2),orient_cand-(pi/2),orient_cand-(pi)];

%%% Find the weighted sum of the prospective errors using sigmoid %%%
%%% Generate Weights %%%
correction = 0;
if ~isempty(orient_hypos)
    b = 10;
    offset = 0.5;
    w = 1./(1+exp(2*b*(abs(orient_hypos) - offset)));
    w_orhy = w.*orient_hypos;
    sw_orhy = sum(w_orhy);
    s_w = sum(w);
    correction = sw_orhy/s_w;
end
%%%
kc = -1;
r_pose
if ~isempty(correction)
    xnz = r_pose(3) + kc*(correction);
   %r_pose(3) = r_pose(3) + kc*(correction);
   
    %correction
   
    %%% LPF
    r_pose(3) = ((b4*xnz + b3*xn(1) + b2*xn(2) + b1*xn(3) + b0*xn(4)) - a3*yn(1) - a2*yn(2) - a1*yn(3) - a0*yn(4))/a4;
   
end
r_pose
xn
yn

xn(4) = xn(3);
xn(3) = xn(2);
xn(2) = xn(1);
xn(1) = xnz;
yn(4) = yn(3);
yn(3) = yn(2);
yn(2) = yn(1);
yn(1) = r_pose(3);