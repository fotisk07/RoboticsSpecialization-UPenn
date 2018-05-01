% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
% myResol = param.resol;
% % the initial map size in pixels

% % the origin of the map in pixels
% myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
% lo_occ = param.lo_occ;
% lo_free = param.lo_free; 
% lo_max = param.lo_max;
% lo_min = param.lo_min;


%pose is 3x1000
%param.resol(25) .size(900,900) .origin(2x1 double) 
% .lo_occ(1) .lo_free(0.5000) .lo_max(100) .lo_min(-100)
% scanAngles the anles of the sensors
% ranges the range of each sensor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
myorigin = param.origin; 
% 
nsensors = length(scanAngles);

% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
for j = 1:N % for each time,
    disp(j)
    xrobot = pose(1,j);
    yrobot = pose(2,j);
    theta = pose(3,j);
    ixrobot = ceil(myResol*xrobot) + myorigin(1);
    iyrobot = ceil(myResol*yrobot) + myorigin(2);
    for sensor = 1:nsensors
        d = ranges(sensor,j);
        alpha = scanAngles(sensor);
        xocc =  d*cos(theta+alpha) + xrobot;
        yocc = -d*sin(theta+alpha) + yrobot;
        ixocc = ceil(myResol*xocc) + myorigin(1);
        iyocc = ceil(myResol*yocc) + myorigin(2);
        indocc = sub2ind(size(myMap), iyocc, ixocc);
        [freex, freey] = bresenham(ixrobot,iyrobot,ixocc,iyocc);
        indfree = sub2ind(size(myMap), freey, freex);
        myMap(indocc) = myMap(indocc) + lo_occ; %%change this
        myMap(indfree) = myMap(indfree) - lo_free; %%change this
    end
    % Find grids hit by the rays (in the gird map coordinate)
     

    % Find occupied-measurement cells and free-measurement cells
   

    % Update the log-odds
  

    % Saturate the log-odd values
    

    % Visualize the map as needed

end
myMap = max(lo_min, min(myMap, lo_max));

end



