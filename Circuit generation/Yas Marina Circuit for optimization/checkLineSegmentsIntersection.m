function intersect = checkLineSegmentsIntersection(x1, y1, x2, y2, x3, y3, x4, y4)
    % Calculate slopes and intercepts for both line segments
    m1 = (y2 - y1) / (x2 - x1);
    b1 = y1 - m1 * x1;
    
    m2 = (y4 - y3) / (x4 - x3);
    b2 = y3 - m2 * x3;
    
    % Check if the segments are parallel
    if abs(m1 - m2) < eps
        intersect = false; % Parallel lines do not intersect
        return;
    end
    
    % Calculate intersection point
    x_intersect = (b2 - b1) / (m1 - m2);
    y_intersect = m1 * x_intersect + b1;
    
    % Check if the intersection point lies within the segments
    intersect = isPointInsideSegment(x_intersect, y_intersect, x1, y1, x2, y2) && isPointInsideSegment(x_intersect, y_intersect, x3, y3, x4, y4);
end


