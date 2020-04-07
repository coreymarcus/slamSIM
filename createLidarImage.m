function image = createLidarImage(imgD, lidarPixelMatches)
%createLidarImage downsamples from a depth image to a lidar image with data
%located at the target points


%intialize image
N = size(lidarPixelMatches,1);
M = size(lidarPixelMatches,2);
image = zeros(N,M);

for ii = 1:N
    for jj = 1:M
        px = lidarPixelMatches(ii,jj,1);
        py = lidarPixelMatches(ii,jj,2);
        image(ii,jj) = imgD(py,px);
    end
end
end