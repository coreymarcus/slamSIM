function I = createImage(C, P, q, V, sz)
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




end

