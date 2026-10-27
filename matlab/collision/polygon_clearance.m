function [distance, pointA, pointB, intersects] = polygon_clearance(A, B, tolerance)
%POLYGON_CLEARANCE Unsigned distance between two closed, simple 2-D polygons.
% Includes edge crossings, collinear overlap, endpoint contact and containment.
% No polyshape/Mapping/Navigation Toolbox dependency. Inputs are N-by-2 vertices.
% Touching or separation <= tolerance is classified conservatively as contact.
if nargin < 3, tolerance = 1e-9; end
validateattributes(A, {'numeric'}, {'2d','ncols',2,'real','finite'});
validateattributes(B, {'numeric'}, {'2d','ncols',2,'real','finite'});
validateattributes(tolerance, {'numeric'}, {'scalar','real','finite','nonnegative'});
if size(A,1)<3 || size(B,1)<3
    error('mir100:InvalidPolygon','Each polygon needs at least three vertices.');
end
A = double(A); B = double(B);
distance = inf; pointA = [NaN NaN]; pointB = pointA;
for i = 1:size(A,1)
    a = A(i,:); b = A(mod(i,size(A,1))+1,:);
    for j = 1:size(B,1)
        c = B(j,:); d = B(mod(j,size(B,1))+1,:);
        [gap,p,q] = segmentGap(a,b,c,d,tolerance);
        if gap < distance
            distance = gap; pointA = p; pointB = q;
        end
        if distance <= tolerance
            distance = 0; intersects = true; return
        end
    end
end
% Segment distances alone miss strict enclosure in either direction.
insideA = inpolygon(A(:,1),A(:,2),B(:,1),B(:,2));
insideB = inpolygon(B(:,1),B(:,2),A(:,1),A(:,2));
if any(insideA)
    pointA = A(find(insideA,1),:); pointB = pointA; distance = 0;
elseif any(insideB)
    pointB = B(find(insideB,1),:); pointA = pointB; distance = 0;
end
intersects = distance <= tolerance;
end

function [gap,p,q] = segmentGap(a,b,c,d,tol)
u = b-a; v = d-c; den = cross2(u,v);
% Relative machine-precision threshold, with correct squared-length units.
denTol = 16*eps*max(norm(u)*norm(v),realmin);
if abs(den) > denTol
    s = cross2(c-a,v)/den; t = cross2(c-a,u)/den;
    if s >= 0 && s <= 1 && t >= 0 && t <= 1
        p = a+s*u; q = p; gap = 0; return
    end
end
% Endpoint projections also handle degenerate edges and parallel overlap.
[q1,g1] = projectPoint(a,c,d);
[q2,g2] = projectPoint(b,c,d);
[p3,g3] = projectPoint(c,a,b);
[p4,g4] = projectPoint(d,a,b);
[gap,k] = min([g1 g2 g3 g4]);
switch k
    case 1, p = a; q = q1;
    case 2, p = b; q = q2;
    case 3, p = p3; q = c;
    otherwise, p = p4; q = d;
end
if gap <= tol, gap = 0; end
end

function [q,gap] = projectPoint(p,a,b)
u = b-a; d = dot(u,u);
if d == 0, q = a;
else, q = a + min(1,max(0,dot(p-a,u)/d))*u;
end
gap = norm(p-q);
end

function z = cross2(a,b)
z = a(1)*b(2)-a(2)*b(1);
end
