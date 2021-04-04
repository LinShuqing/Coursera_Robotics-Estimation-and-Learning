% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


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
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

% 定位次数
N = size(pose,2);
% 一次定位下的激光雷达数据数目
M = size(ranges, 1);
% return a 2D array map
for j = 1:N % for each time,
%  机器人坐标、角度
    pose_x=pose(1,j);
    pose_y=pose(2,j);
    pose_theta=pose(3,j);
    
% 机器人坐标计算(根据文档，是乘以myResol)
    x_start=ceil(pose_x*myResol)+myorigin(1);
    y_start=ceil(pose_y*myResol)+myorigin(2);
      
%     % Find grids hit by the rays (in the gird map coordinate)
    for i=1:M
        beta = pose_theta +scanAngles(i);
        d = ranges(i,j);
        x_loc=d*cos(beta)+pose_x;
        y_loc=-d*sin(beta)+pose_y;
%     % Find occupied-measurement cells and free-measurement cells
        i_x_occ=ceil(x_loc*myResol)+myorigin(1);
        i_y_occ=ceil(y_loc*myResol)+myorigin(2);
        
        [free_x,free_y] = bresenham(x_start,y_start,i_x_occ,i_y_occ);
        
        free = sub2ind(size(myMap),free_y , free_x );
        occ = sub2ind(size(myMap), i_y_occ , i_x_occ);
% 
%     % Update the log-odds 更新地图
         myMap(free) = myMap(free) - lo_free;
         myMap(occ) = myMap(occ) + lo_occ;
    end
% 
%     % Saturate the log-odd values
    myMap(myMap < lo_min) = lo_min;
    myMap(myMap > lo_max) = lo_max;
% 
%     % Visualize the map as needed
%    
% 
end

end

