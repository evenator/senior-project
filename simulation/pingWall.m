function [pingDistance] = pingWall(angle, x, y, heading, map)

x1 = x; y1 = y;
x2 = 40000*cos(angle+heading - 2.09) + x; y2 = 40000*sin(angle+heading-2.09) + y;

tmp = size(map);
pingDistance = 10000000;

for i = 2:tmp(1) 
    
    x3 = map(i, 1); y3 = map(i, 2);
    x4 = map(i-1, 1); y4 = map(i-1,2);
    
    denom = (y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1);
    
    if (denom ~= 0)
        ua = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / denom;
        ub = ((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) / denom;
        x_i = x1 + ua*(x2 - x1);
        y_i = y1 + ua*(y2 - y1);
        
        p = sqrt( (x-x_i)^2 + (y-y_i)^2 );
        
        if (p < pingDistance && all(([ua ub] >= 0) & ([ua ub] <= 1)) ) 
            pingDistance = p;
        end
    end
    
end

end

