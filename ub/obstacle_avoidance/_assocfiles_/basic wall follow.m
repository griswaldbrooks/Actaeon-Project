clear; clc; cla; hold all; colormap white; set( gca, 'color', 'k' ); axis( 200*[0,1,0,1] )
[world world_mat]= trinitymap();
% -------------------------------------------------------------------------------------- %




% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %
heading     = [ 0; 1 ];
position    = [ 100; 20 ];
robotwidth  = 10;
omega       = 0;
veloc       = .5;

PID_response  = .25;

laserhead   = 0;
nscans      = 360
laserfreq   = (2*pi/(nscans-1));
laserradius = 300;

new_omega   = 0;
new_veloc   = .4;

drift       = pi/5000;
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %









% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %
% ALG VARS

CLR   = 30;
SCOPE = 50;

% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %



ROUND = 0;
while(1)
cla
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %
ranges = [];
angular = -atan2(heading(1),heading(2));
while( laserhead < 2*pi-(0.01) )
[laserhead, ranges(:,end+1)] = lasermodel(position,angular,laserhead,laserfreq,laserradius,world_mat,200);
end
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %

width  = 10;

%---+ leveling +---%
level_left  = sum(ranges(2,045:135));
level_right = sum(ranges(2,225:315));
level_total = level_left-level_right;
passalong   = (((level_left + level_right)/90)/150)

new_omega   = level_total*(1-passalong)/(5E4);

%---+ navigation +---%
push_left   = sum(ranges(2,001:045));
push_right  = sum(ranges(2,315:360));

new_omega   = new_omega + (push_left-push_right)*(passalong)/(5E4);
new_veloc   = ((sum(ranges(2,001:020)) + sum(ranges(2,340:360)))/40 - width)/(1E2);




%---+ lookout +---%


















% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %
omega = omega + (new_omega-omega+(drift*randn(1)))*PID_response;
veloc = veloc + 2*(new_veloc-veloc)*PID_response;

    new_veloc = 0;
    new_omega = 0;


heading  = [cos(omega), -sin(omega) ; sin(omega), cos(omega) ]*heading;
position = position + veloc*heading*2;
angle = atan2(heading(1),heading(2));
plot( position(1),   position(2),   'yo' )
plot( world(1,2:end), world(2,2:end), 'w.', 'markersize', 2 )

ranges = [];
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %

pause(1E-3)
end

