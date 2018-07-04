function [n, d] = pldist(p, p1, p2)

% convert 2d points to 3d if necessary
if size(p,1) == 2 && size(p,2) == 1
    p  = [p; 0];
    p1 = [p1; 0];
    p2 = [p2; 0];
    is2D = true;
elseif size(p,1) == 1 && size(p,2) == 2
    p  = [p, 0];
    p1 = [p1, 0];
    p2 = [p2, 0];
    is2D = true;
else
    is2D = false;
end

% find the vector n
e = (p2-p1)/norm(p2-p1);
n = (p-p1) - e*dot(p-p1,e);

% reduce to 2d and compute dist
if is2D == true
    n = n(1:2);
end
d = norm(n);
n = n/d;

end