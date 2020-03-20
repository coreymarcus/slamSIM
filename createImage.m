function I = createImage(C, P, q, V, sz, K)
%creates an RGB image of a cube with a white background given cube
%parameters, camera position and orientation, and camera calibration matrix
% Inputs
% C = cube structure
% P = [3x1] camera position in inertial frame
% q = [4x1] quaternion rotating from inertial to camera frame, scalar firsts
% V = [height x width x 3] matrix of vectors corresponding to each pixels
% sz = [2x1] = [width, height] image size in pixels
% 
% Outputs
% I = [height x width x 3] RGB triplet information


%initialize image
width = sz(1);
height = sz(2);
I = ones(height, width, 3);

%locate the verticies of the cube in pixel form
Verts = locateVerticies(C, P, q, K);

xMin = max([Verts(1) 1]);
xMax = min([Verts(2) width]);
yMin = max([Verts(3) 1]);
yMax = min([Verts(4) height]);

% cycle throught all the points
parfor ii = xMin:xMax
    for jj = yMin:yMax
        
        %get pixel vector and then rotate it into the inertial frame
        v = squeeze(V(jj,ii,:));
        v = quatrotate(quatconj(q'),v')';
        
        % check intersect
        inter = checkIntersect(C, P, v);
        
        if(inter == 0) %we hit nothing
            continue;
        elseif(inter == 7) %we hit an edge
            I(jj,ii,:) = [0 0 0];
            continue;
        end
        
        %else we hit an edge and need to get that edges color
        I(jj,ii,:) = C.faces{inter}.color;
        
    end
end



end

