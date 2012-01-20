function over = overlap(land1,land2)
%%% Landmarks must be of the same orientation
    over = 0;
    dy = land1(2) - land1(4);
    dx = land1(1) - land1(3);
    angle = abs(atan2(dy,dx));
    if (angle == 0) || (angle == pi)
        if (land1(1) > land2(1)) && (land1(1) < land2(3))
            over = 1;
        elseif (land1(1) < land2(1)) && (land1(1) > land2(3))
            over = 1;
        elseif (land1(3) > land2(1)) && (land1(3) < land2(3))
            over = 1;
        elseif (land1(3) < land2(1)) && (land1(3) > land2(3))
            over = 1;
        end
    else
        if (land1(2) > land2(2)) && (land1(2) < land2(4))
            over = 1;
        elseif (land1(2) < land2(2)) && (land1(2) > land2(4))
            over = 1;
        elseif (land1(4) > land2(2)) && (land1(4) < land2(4))
            over = 1;
        elseif (land1(4) < land2(2)) && (land1(4) > land2(4))
            over = 1;
        end
    end