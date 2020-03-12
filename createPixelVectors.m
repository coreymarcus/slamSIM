function V = createPixelVectors(K, width, height)
% Creates a bunch of vectors corresponding to each pixel in a image
% Inputs
% K = [3x3] camera calibration matrix
% width = [1x1] image width in pixels
% height = [1x1] image height in pixels
% 
% Outputs
% I = [height x width x 3] RGB triplet information

V = zeros(height, width, 3);
Kinv = K^(-1);

for ii = 1:width
    for jj = 1:height
        p = [ii jj 1]';
        x = Kinv*p;
        V(jj,ii,:) = x/norm(x);
    end
end

end

