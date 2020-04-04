function Verts = locateVerticies(C, P, q, K)
%LOCATEVERTICIES finds the min and max pixels which will be occupied by the
%cube to speed up image generation
% 
% Inputs
% C - cube structure
% P - [3x1] camera position in world frame
% q = [4x1] quaternion rotating from inertial to camera frame, scalar firsts
% K = [3x3] camera calibration matrix
% 
% Outputs
% Verts = [pixXmin, pixXmax, pixYmin, pixYmax]

%initialize points
pts = zeros(6*4,2);

%really a cube has 8 unique verticies, but I'm not sure how to identify
%them so we'll go through all of the potentials
for ii = 1:6
    %extract points
    ptsInWrld = C.faces{ii}.vertex;
    
    %translate to camera
    ptsWrtCam = ptsInWrld - [P P P P];
    
    %rotate into camera frame
    ptsInCam = quatrotateCoder(q',ptsWrtCam')';
    
    %find pixels
    for jj = 1:4
        x = K*ptsInCam(:,jj);
        p = x(1:2)/x(3);
        pts((ii-1)*4 + jj,:) = round(p)';
    end
    
end

%write output
Verts = zeros(1,4);

Verts(1) = min(pts(:,1)) - 1;
Verts(2) = max(pts(:,1)) + 1;
Verts(3) = min(pts(:,2)) - 1;
Verts(4) = max(pts(:,2)) + 2;

end

