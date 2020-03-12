function I = checkIntersect(C, P, v)
%checkIntersect detirmines if a vector will intersect the cube
% Inputs
% C - cube structure
% P - 3x1 point of origin for the vector
% v = 3x1 vector direction
% 
% Outputs
% I = 0 if no intersect, i if hitting the ith face (1:6), 7 if hitting an
%   edge

ints = zeros(6,1);
edgeHit = zeros(6,1);
for ii = 1:6
    %extract face information
    F = C.faces{ii};
    
    %our strategy is to solve for when the vector passes through the plane
    %of each face
    if(strcmp(F.plane,'x'))
        t = (F.center(1) - P(1))/v(1);
    elseif(strcmp(F.plane,'y'))
        t = (F.center(2) - P(2))/v(2);        
    elseif(strcmp(F.plane,'z'))
        t = (F.center(3) - P(3))/v(3);
    end
    
    %throw out bad intersects
    if(isnan(t) || isinf(t) || (t <= 0))
        continue;
    end
    
    %check to see if the intersect is actually on the face or if is on an
    %edge
    intPt = P + t*v;
    
    if(strcmp(F.plane,'x'))
        if (intPt(2) > max(F.vertex(2,:)))
            continue;
        end
        
        if (intPt(2) < min(F.vertex(2,:)))
            continue;
        end
        
        if (intPt(3) > max(F.vertex(3,:)))
            continue;
        end
        
        if (intPt(3) < min(F.vertex(3,:)))
            continue;
        end
        
        if (intPt(2) > max(F.vertex(2,:) - C.sw ))
            edgeHit(ii) = 1;
        end
        
        if (intPt(2) < min(F.vertex(2,:) + C.sw))
            edgeHit(ii) = 1;
        end
        
        if (intPt(3) > max(F.vertex(3,:) - C.sw))
            edgeHit(ii) = 1;
        end
        
        if (intPt(3) < min(F.vertex(3,:) + C.sw))
            edgeHit(ii) = 1;
        end
        
    elseif(strcmp(F.plane,'y'))
        
        if (intPt(1) > max(F.vertex(1,:)))
            continue;
        end
        
        if (intPt(1) < min(F.vertex(1,:)))
            continue;
        end
        
        if (intPt(3) > max(F.vertex(3,:)))
            continue;
        end
        
        if (intPt(3) < min(F.vertex(3,:)))
            continue;
        end
        
        if (intPt(1) > max(F.vertex(1,:) - C.sw))
           edgeHit(ii) = 1;
        end
        
        if (intPt(1) < min(F.vertex(1,:) + C.sw))
            edgeHit(ii) = 1;
        end
        
        if (intPt(3) > max(F.vertex(3,:) - C.sw))
            edgeHit(ii) = 1;
        end
        
        if (intPt(3) < min(F.vertex(3,:) + C.sw))
            edgeHit(ii) = 1;
        end
                
    elseif(strcmp(F.plane,'z'))
        
        if (intPt(2) > max(F.vertex(2,:)))
            continue;
        end
        
        if (intPt(2) < min(F.vertex(2,:)))
            continue;
        end
        
        if (intPt(1) > max(F.vertex(1,:)))
            continue;
        end
        
        if (intPt(1) < min(F.vertex(1,:)))
            continue;
        end
        
        if (intPt(2) > max(F.vertex(2,:) - C.sw))
            edgeHit(ii) = 1;
        end
        
        if (intPt(2) < min(F.vertex(2,:) + C.sw))
            edgeHit(ii) = 1;
        end
        
        if (intPt(1) > max(F.vertex(1,:) - C.sw))
            edgeHit(ii) = 1;
        end
        
        if (intPt(1) < min(F.vertex(1,:) + C.sw))
            edgeHit(ii) = 1;
        end
        
    end
    
    ints(ii) = t;
    
end
ints

%if theres no intersect, return 0
if(norm(ints) == 0)
    I = 0;
    return;
end

[Y, ~] = min(ints(ints > 0));

I = find(ints == Y);
if(length(I) > 1)
    I = I(1);
    disp('Exact Edge Intersect');
end

%check for edge hit
if(edgeHit(I) == 1)
    I = 7;
end

end

