function [drone_x, drone_y, drone_heading] = moveDrone(x, y, heading, vel_x, vel_y, vel_heading, dt)

    drone_x = x + vel_x * dt;
    drone_y = y + vel_y * dt;
    drone_heading = heading + vel_heading * dt;

end

