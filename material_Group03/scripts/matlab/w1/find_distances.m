function [d, pts] = find_distances(points, x, y)
    d = zeros(length(points(:, 1))-1, 1);
    pts = zeros(length(points(:, 1))-1, 2);
    z = [x y];

    for i=1:(length(points(:, 1))-1)
        x1 = points(i,:);
        x2 = points(i+1,:);
        vline = x2 - x1;
        x1_z = z - x1;
        x1_p = vline*(x1_z*vline'/(norm(vline)^2));
        s = x1_z*vline'/(norm(vline)^2); % 0 - 1
        p = x1 + x1_p;
        
        if isbetween(s, 0, 1)
            d(i) = norm(p - z);
        elseif s > 1
            d(i) = norm(z - x2);
        else
            d(i) = norm(z - x1);
        end
            
    end
end