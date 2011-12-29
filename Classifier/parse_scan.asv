function vv_points = parse_scan(laser_rth)

vv_points = [];

    SEP_THRESH = 12.5; %cm
    MIN_NUM_OF_RANGES = 3; %Minimum number of ranges needed to produce points
    vect_ranges = zeros(length(laser_rth),2);
    vect_points = zeros(length(laser_rth),2);
    
    for ndx = 1:length(laser_rth)
      % If the ranges are similar to each other then they are candidates
      if abs(laser_rth(ndx,1) - laser_rth(ndx + 1,1)) < SEP_THRESH
	  % If this is the first time anything has been added, add both
    	  if sum(vect_ranges(:,1)) == 0
              vect_ranges(ndx,1) = laser_rth(ndx,1);
              vect_ranges(ndx,2) = laser_rth(ndx,2);
          end	  
          vect_ranges(ndx+1,1) = laser_rth(ndx+1,1);
          vect_ranges(ndx+1,2) = laser_rth(ndx+1,2);
	
      % If we are not adding ranges and there are more than two candidate ranges,
      % then add them to the points vector
      elseif sum(vect_ranges(:,1)~=0) >= MIN_NUM_OF_RANGES
    	for ndx = 1:length(vect_ranges)
            x = vect_ranges(ndx,1)*cos(vect_ranges(ndx,2)) + r_pose(1);
            y = vect_ranges(ndx,1)*sin(vect_ranges(ndx,2)) + r_pose(2);
            vect_points(ndx,1) = x;
            vect_points(ndx,2) = y;
        end
    	vv_points = [vv_points, vect_points];
        vect_ranges = zeros(length(laser_rth),2);
        vect_points = zeros(length(laser_rth),2);
      
      % If we are not adding ranges, and the current range vector contains too few candidates
      else
        vect_ranges = zeros(length(laser_rth),2);
      end
     
      %Check the special case of an object being split by the beginning and end of the scan
      if ndx == length(laser_rth)
        if abs(laser_rth(ndx,1) - laser_rth(1,1)) < SEP_THRESH
        {
            
	    vect_ranges.push_back(RT(scan.ranges[ndx + 1], scan.angle_increment*ndx + scan.angle_min));
	    for(size_t iter = 1; iter < (scan.ranges.size() - 1); iter++){
            if((fabs(scan.ranges[iter] - scan.ranges[iter + 1]) < SEP_THRESH){
		  % If this is the first time anything has been added, add both
    		  if(vect_ranges.size() == 0){
        	    vect_ranges.push_back(RT(scan.ranges[iter], scan.angle_increment*iter + scan.angle_min));
              }
            vect_ranges.push_back(RT(scan.ranges[iter + 1], scan.angle_increment*iter + scan.angle_min));

		  %std::cout << "Pushed range: " << scan.ranges[ndx + 1] << std::endl;
            }
	      % If we are not adding ranges and there are more than two candidate ranges,
	      % then add them to the points vector
        else if(vect_ranges.size() >= MIN_NUM_OF_RANGES){
    		for(size_t ndx = 0; ndx < vect_ranges.size(); ndx++){
        	  float x = vect_ranges[ndx].radius*cos(vect_ranges[ndx].theta);
              float y = vect_ranges[ndx].radius*sin(vect_ranges[ndx].theta);
    		  %Push the point in cm
        	  vect_points.push_back(Point(x*100,y*100));
            }
    		vv_pts.push_back(vect_points);
        	vect_points.clear();
    		vect_ranges.clear();
              }
	      % If we are not adding ranges, and the current range vector contains too few candidates
	      else{
		break;
	      }
	    }
	  }
      } 
    }
    
  }