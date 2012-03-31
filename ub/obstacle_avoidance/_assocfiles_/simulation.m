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

use_error_right = 0;
use_error_left  = 0;

% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %



ROUND = 0;
while(1)
cla
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %
ranges = [];
angular = -atan2(heading(1),heading(2));
[laserhead, ~] = lasermodel(position,angular,laserhead,laserfreq,laserradius,world_mat,200);
while( laserhead > 0 )
[laserhead, ranges(:,end+1)] = lasermodel(position,angular,laserhead,laserfreq,laserradius,world_mat,200);
end
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %





% sum of errors
width      = 15;
hallwidth  = 26;
standard  = (hallwidth/cos(45)+width)/2;

error_left = 0;
for( l = 45:135 )
     error_left = error_left + abs(standard*cosd(l))-ranges(2,l);
end

error_right = 0;
for( r = 225:315 )
    error_right = error_right + abs(standard*cosd(r))-ranges(2,r);   
end

error_foward = 0;
for( r = 1:20 )
    error_foward = error_foward + ranges(2,r)+ranges(2,359-r);   
end

new_veloc =(error_foward/60 - width)/(2*width);
new_omega = 0;
new_omega = new_omega-(error_left-width)/(5E5);
new_omega = new_omega+(error_right-width)/(5E5);

necessity = exp(-abs(error_left+error_right)/width^2)
new_omega = new_omega+necessity*(-error_left + error_right)/(5E5);






















% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %
omega = omega + (new_omega-omega+(drift*randn(1)))*PID_response;
veloc = veloc + 2*(new_veloc-veloc)*PID_response;

    new_veloc = 0;
    new_omega = 0;


heading  = [cos(omega), -sin(omega) ; sin(omega), cos(omega) ]*heading;
position = position + veloc*heading/1.5;
angle = atan2(heading(1),heading(2));
plot( position(1),   position(2),   'yo' )
plot( world(1,2:end), world(2,2:end), 'w.', 'markersize', 2 )

ranges = [];
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ %

pause(1E-3)
end

