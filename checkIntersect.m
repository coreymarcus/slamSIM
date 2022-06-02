function [I, D, pt] = checkIntersect(C, P, v_inertial, v_cam)
%checkIntersect detirmines if a vector will intersect the cube
% Inputs
% C - cube structure
% P - 3x1 point of origin for the vector
% v_intertial = 3x1 vector direction in inertial frame
% v_cam = 3x1 vector direction in camera frame
% 
% Outputs
% I = 0 if no intersect, i if hitting the ith face (1:6), 7 if hitting an
%   edge
% D = depth to cube, 0 if no intersect
% pt = [2x1] intersect point on the cube face where each element in [0, 1]


%initialize depth
D = 0;

%initialize intersect point
pt = [0 0]';

%initialize detection logic
ints = zeros(6,1);
edgeHit = zeros(6,1);
pts = zeros(2,6);
for ii = 1:6
    %extract face information
    F = C.faces{ii};
    
    %our strategy is to solve for when the vector passes through the plane
    %of each face
    if(strcmp(F.plane,'x'))
        t = (F.center(1) - P(1))/v_inertial(1);
    elseif(strcmp(F.plane,'y'))
        t = (F.center(2) - P(2))/v_inertial(2);        
    else %z plane
        t = (F.center(3) - P(3))/v_inertial(3);
    end
    
    %throw out bad intersects
    if(isnan(t) || isinf(t) || (t <= 0))
        continue;
    end
    
    %check to see if the intersect is actually on the face or if is on an
    %edge
    intPt = P + t*v_inertial;
    
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
    
    % If we're here, we have intersected this cube
    ints(ii) = t;

    % Find point wrt cube center
    ptrelcube = intPt - C.P;

    % Find the location on the cube face
    switch F.plane
        case 'x'
            ptrelface = ptrelcube(2:3);
        case 'y'
            ptrelface = ptrelcube([1 3]);
        case 'z'
            ptrelface = ptrelcube(1:2);
        otherwise
            error('Invalid F.plane in checkIntersect()');
    end

    % Normalize and shift ptrelface to [0,1] domain
    ptrelface_normalized = (ptrelface + [C.s(1)/2 C.s(1)/2]')./C.s(1);

    pts(:,ii) = ptrelface_normalized;
end

%if theres no intersect, return 0
if(norm(ints) == 0)
    I = 0;
    return;
end

[Y, ~] = min(ints(ints > 0));

I = find(ints == Y);
if(length(I) > 1)
    I = I(1);
    % disp('Exact Edge Intersect');
end

% Assign point
pt = pts(:,I);

%check for edge hit
if(edgeHit(I) == 1)
    I = 7;
end

%assign distance
D = Y*v_cam(3);

%make sure I is one dimensional
I = I(1);

end

