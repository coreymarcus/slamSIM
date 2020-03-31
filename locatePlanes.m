function boxes = locatePlanes(C, P, q, K)
%LOCATEPLANES finds the verticies of a series of planes corresponding to the
%visible faces of a cube (1, 2, or 3).
% 
% Inputs
% C - cube structure
% P - [3x1] camera position in world frame
% q = [4x1] quaternion rotating from inertial to camera frame, scalar firsts
% K = [3x3] camera calibration matrix
% 
% Outputs
% boxes = [Nx16] matrix containing information on each of the N visible
%   planes. A row is divided into [Edge Info, Plane Info], where the edge
%   info talks about the border of the cube and the plane info about the
%   central plane. A given info is formated like this [x1 y1 x2 y2 x3 y3 x4
%   y4] where the first point corresponds to the point closest to the
%   origin in the image and then incremented clockwise around the shape


%detirmine how many sides will be visible
%find vectors to the center of each face
ptsInCam = zeros(3,6);
xNorm = zeros(1,6);
yNorm = zeros(1,6);
for ii = 1:6
    %extract points
    ptInWrld = C.faces{ii}.center;
    
    %translate to camera
    ptsWrtCam = ptInWrld - P;
    
    %rotate into camera frame
    ptsInCam(:,ii) = quatrotate(q',ptsWrtCam')';
    
    xNorm(ii) = abs(ptsInCam(1,ii));
    yNorm(ii) = abs(ptsInCam(2,ii));
   
end

%if the norm of any of these vectors in x AND y is zero, only one plane is
%visible and it is the closer of the two
xyNorm = xNorm+yNorm;
Ixy = find(xyNorm == 0);
if(~isempty(Ixy))
    %only one side is going to be visible
    N = 1
end

%now check to see if maybe only two sides are visible
Ix = find(xNorm == 0);
Iy = find(yNorm == 0);

if(~isempty(Ix) || ~isempty(Iy))
    %two sides are visible
    N = 2
end


end

