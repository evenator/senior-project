% Main simulation file

% Define simulation parameters

% Define the system's kinematic parameters
drone_x = 25; drone_y = 25; drone_heading = 1;
bitmap = -1 * ones(60);

map = [[10,10]; [50, 10]; [40, 40]; [10, 50]; [10,10]];
figure(1);
drawMap(map, drone_x, drone_y, drone_heading);

for t = 1:10 % Adjust the end to get more movement in the convex-polygon (suggest 50 for coolness)
    for lidar_heading=0:0.0062832:4.19
         minDistance = pingWall(lidar_heading, drone_x, drone_y, drone_heading, map);
         
         % This is the algorithm for confidence mapping
         ping_x = drone_x + minDistance*cos(drone_heading+lidar_heading-2.09);
         ping_y = drone_y + minDistance*sin(drone_heading+lidar_heading-2.09);
         
         x_0 = drone_x; y_0 = drone_y;
         x_f = ping_x; y_f = ping_y;
                  
         if (round(x_0) == round(x_f)) % Deal with infinite slop in linear interpolation
            for y=y_0:-(abs(y_0-y_f)/(y_0-y_f)):y_f
               bx = round(x_0); by = round(y);
               if (bitmap(bx,by) <= 0) 
                   bitmap(bx,by) = 0;
               else
                   bitmap(bx,by) = bitmap(bx,by) - .1;           
               end        
            end
         else
            for x=x_0:-(abs(x_0-x_f)/(x_0-x_f)):x_f
               y = y_0 + (x-x_0)*( (y_f-y_0)/(x_f - x_0) );
               bx = round(x); by = round(y);
               if (bitmap(bx,by) <= 0) 
                   bitmap(bx,by) = 0;
               else
                   bitmap(bx,by) = bitmap(bx,by) - .1;           
               end
            end
         end
         plot([drone_x ping_x], [drone_y ping_y], 'k');
         
         bitmap(round(drone_x), round(drone_y)) = -2;
         if (bitmap(round(ping_x),round(ping_y)) < 0.9 && bitmap(round(ping_x),round(ping_y)) >= 0)
             bitmap(round(ping_x), round(ping_y)) = bitmap(round(ping_x),round(ping_y)) + 0.1;
         end
         % End confidence mapping
             
    end
    
    [drone_x, drone_y, drone_heading] = moveDrone(drone_x, drone_y, drone_heading, 0, 0, -.3, 0.1);
    drawMap(map, drone_x, drone_y, drone_heading);
    t
end


figure(2); hold all;
fill(1,1,'k');
for x=1:60
    for y=1:60
        if bitmap(x,y) == -1
            plot(x,y,'s', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
        elseif bitmap(x,y) == 0
            plot(x,y,'s', 'MarkerFaceColor', [1 1 1], 'MarkerEdgeColor', [1 1 1]);
        elseif bitmap(x,y) == -2
            plot(x,y,'s', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
        else
            c = (1-bitmap(x,y)) * [1 1 1];
            plot(x,y,'s', 'MarkerFaceColor', c, 'MarkerEdgeColor', c);
        end
        
    end
end


    

