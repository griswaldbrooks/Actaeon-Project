function closeto = close_x_or_y(land1,land2,thresh)
%%% Landmarks must be of the same orientation
    closeto = 0;
    dy = land1(2) - land1(4);
    dx = land1(1) - land1(3);
    angle = abs(atan2(dy,dx));
    if (angle == 0) || (angle == pi)
        error = abs(land1(5) - land2(5));
    else
        error = abs(land1(6) - land2(6));
    end
    
    if error < thresh
        closeto = 1;
    end