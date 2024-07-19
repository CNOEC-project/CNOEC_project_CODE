function inside = isPointInsideSegment(x, y, x1, y1, x2, y2)
    inside = (x >= min(x1, x2) && x <= max(x1, x2)+3) && ...
             (y >= min(y1, y2) && y <= max(y1, y2));
end