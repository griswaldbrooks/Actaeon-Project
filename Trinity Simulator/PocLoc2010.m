function [v, om] = PocLoc2010(laser_rp)
denom_n90 = laser_rp(28)^2;
    denom_p90 = laser_rp(10)^2;
	denom_n45 = laser_rp(32)^2;
	denom_p45 = laser_rp(6)^2;
    
    if(denom_n90 ~= 0)
    	cor_ang_n90 =  1000.0/denom_n90;
		if (cor_ang_n90 > 90)
            cor_ang_n90 = 90;
        end
		if(cor_ang_n90 < -90)
            cor_ang_n90 = -90;
        end
    
    else cor_ang_n90 = 90;
    end

	if(denom_p90 ~= 0)
		cor_ang_p90 =  1000.0/denom_p90;
		if(cor_ang_p90 > 90)
             cor_ang_p90 = 90;
        end
		if(cor_ang_p90 < -90)
             cor_ang_p90 = -90;
        end
    else cor_ang_p90 = 90;
    end

    if(denom_n45 ~= 0)
		cor_ang_n45 =  1000.0/denom_n45;
		if(cor_ang_n45 > 90)
            cor_ang_n45 = 90;
        end
		if(cor_ang_n45 < -90)
            cor_ang_n45 = -90;
        end
    else cor_ang_n45 = 90;
    end

	if(denom_p45 ~= 0)
		cor_ang_p45 =  1000.0/denom_p45;
		if(cor_ang_p45 > 90)
            cor_ang_p45 = 90;
        end
		if(cor_ang_p45 < -90)
            cor_ang_p45 = -90;
        end
    else cor_ang_p45 = 90;
    end
    
    cor_coeff = 40/laser_rp(1);
    
    om = (cor_coeff)*((cor_ang_p90 - cor_ang_n90) + (cor_ang_p45 - cor_ang_n45))*(pi/180);
    v = sqrt(laser_rp(1)/40);