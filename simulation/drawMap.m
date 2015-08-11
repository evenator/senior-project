% Draw the contents of the map

function drawMap(map, drone_x, drone_y, drone_heading)

hold all;

% Draw frame and map
plot([0, 0, 60, 60], [0, 60, 60, 0]);
%plot(map(:,1), map(:,2));

% Draw drone
plot(drone_x, drone_y, 's', 'MarkerSize', 2, 'MarkerFaceColor', 'g');
plot([drone_x, 4*cos(drone_heading)+drone_x], [drone_y, 4*sin(drone_heading)+drone_y]);

end




