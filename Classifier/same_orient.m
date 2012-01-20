function same = same_orient(land1,land2)
    dy1 = land1(2) - land1(4);
    dx1 = land1(1) - land1(3);
    angle1 = abs(atan2(dy1,dx1));
    dy2 = land2(2) - land2(4);
    dx2 = land2(1) - land2(3);
    angle2 = abs(atan2(dy2,dx2));
    same = (angle1 == angle2) || (angle1 == (angle2 - pi));