function [pointset, mat] = trinitymap()
% generate map:
% -------------------------------------------------------------------------------------- %
radius  = 100;
hallwid = 13;
roomwid = 80;
aptrwid = 15;
midroom = 52;
world  = ones(2*radius);
for i = 1:2*radius
    world(i,(radius-hallwid):(radius+hallwid)) = 0;
    world((radius-hallwid):(radius+hallwid),i) = 0;
end
for i = 1:radius
    world(2*radius-i,(2*(radius-hallwid)):2*(radius)) = 0;
    world((2*(radius-hallwid)):2*(radius),2*radius-i) = 0;
end
pin1 = 3;  world((pin1:pin1+roomwid),(pin1:pin1+roomwid)) = 0;
pin2 = 3;  world(2*radius-(pin2:pin2+roomwid),(pin2:pin2+roomwid)) = 0;
pin3 = 3;  world((pin3:pin3+roomwid),2*radius-(pin3:pin3+roomwid)) = 0;
pin4 = 31; world(2*radius-(pin4:pin4+midroom),2*radius-(pin4:pin4+midroom)) = 0;
% room 1 opening
rx1  = 110;
ry1  = 20;
world( (rx1-aptrwid):(rx1+aptrwid), (ry1-aptrwid):(ry1+aptrwid) ) = 0;
% room 2 opening
rx2  = 20;
ry2  = 110;
world( (rx2-aptrwid):(rx2+aptrwid), (ry2-aptrwid):(ry2+aptrwid) ) = 0;
% room 3 opening
rx3  = 65;
ry3  = 90;
world( (rx3-aptrwid):(rx3+aptrwid), (ry3-aptrwid):(ry3+aptrwid) ) = 0;
% room 4 opening
rx4  = 150;
ry4  = 120;
rim = ones(200);
rim( 4:end-3, 4:end-3) = 0;
world( (rx4-aptrwid):(rx4+aptrwid), (ry4-aptrwid):(ry4+aptrwid) ) = 0;
world = world(end:-1:1,end:-1:1);
mat   = world;
mat   = mat + rim;
mat   = boolean(mat);
pointset = zeros(2,1); 

for( i = 1:200 )
    for( j = 1:200 )
        if mat(i,j) > 0
            pointset(:,end+1) = [i;j];
        end
    end
end
    
end