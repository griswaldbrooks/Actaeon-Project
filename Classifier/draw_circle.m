function draw_circle(xc,yc,r,c)
num_points = 16;
angle_inc = (2*pi)/num_points;
points = repmat(0,num_points,2);
for ang_iter = 1:num_points
    x = r*cos(ang_iter*angle_inc) + xc;
    y = r*sin(ang_iter*angle_inc) + yc;
    points(ang_iter,:) = [x,y];
end
points = [points;points(1,:)];
line(points(:,1),points(:,2),'Color', c)