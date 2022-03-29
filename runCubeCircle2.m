% Runs a circle around a cube

clear
close all
clc


%% Options

%make sure we get a different seed every time
rng('shuffle');

% LIDAR
LidarFOVHeight = pi/6; %radians
LidarFOVWidth = pi/4; %radians
LidarArrayWidth = 50;
LidarArrayHeight = 50;
lidarImageRate = 1; %one to create lidar image for every camera image

%circle parameters
R = 6; %radius
Nrev = 10000; %number of images per revolution
inc = .5; %magnitude of oscilations
Revs = .5; %number of revolutions around the cube
Noscil = 70; %number of oscillations per revolution
N = round(Revs*Nrev);

%control if we used compiled code for image generation
useMexForImgGen = false;

% Should we create lidar images?
createlidar = false;

%true depth data is massive, run this if you only want to create and save
%it at the target index
savetruth = false;
targIdx = [1:1000]; %set of frames we'd like to run (1-idx, not 0-idx)
targKF = targIdx(1); %the frame where dense depth data for each image pixel will be saved
runTargOnly = true;

%savepath for data
% savepath = 'C:\Users\corey\Documents\SharedFolder\truth';
savepath = 'data/';

%save truth information as csv?
saveAsCsv = true;

% Noise
addRBGNoise = false;
addDepthNoise = false;
addTrajNoise = false;
MuLidar = 0; %average lidar depth noise
PLidar = .01^2; % lidar depth covariance, m^2
MuRGB = 0; % average RGB noise
PRGB = .01; % RGB noise covariance
GaussBlurFactor = 1;
MuPos = zeros(3,1); % average position noise
PPos = .001*eye(3); % position noise covariance
MuEul = zeros(3,1); % Average euler angle noise
PEul = .0001*eye(3);

% Parallel Pool
% pool = gcp('nocreate')
% if(isempty(pool))
%     pool = parpool(6)
%     addAttachedFiles(pool, 'createImage_mex.mexw64')
% end

% Scale for output translation
scale_slam2truth = 2.5410;

% Output file name for tracking truth
outputfile = 'C:\Users\corey\Documents\SharedFolder\SLAMsim_data\truth\TrackingTruth.csv';

%% Main
%path for wahba solver
addpath('../matlabScripts/')

theta = linspace(0,2*pi*Revs,N);
% phi = (theta/(6*Revs)).*sin(3*theta) + inc*sin(Noscil*theta);
phi = inc*sin(Noscil*theta);
x = zeros(3,N);
for ii = 1:N
    x(1,ii) = R*cos(theta(ii))*cos(phi(ii));
    x(2,ii) = R*sin(theta(ii))*cos(phi(ii));
    x(3,ii) = R*sin(phi(ii));
    %     x(3,ii) = inc;
end

if(addTrajNoise)
    disp('Adding attitude noise early!!!')
    posnoise = mvnrnd(MuPos',PPos,N);
    x = x + posnoise';
end

% figure
% scatter3(x(1,:),x(2,:),x(3,:))
% axis equal

% Create the cube
P = [1; 0; 0];
s = 0.5;
sw = 0.05;
fc = [1 0 0;
    0 1 0;
    0 0 1;
    1 1 0;
    1 0 1;
    0 1 1];

ec = [.1 .1 .1];
C = createCube(P, s, sw, fc, ec);

%create more cubes
P2 = [0; 0; -2];
C2 = createCube(P2, s, sw, fc, ec);

P3 = [1; -1; 0];
C3 = createCube(P3, s, sw, fc, ec);

P4 = [0; 2; .5];
C4 = createCube(P4, s, sw, fc, ec);

P5 = [-1; 0; 0];
C5 = createCube(P5, s, sw, fc, ec);

P6 = [.4; -1; .4];
C6 = createCube(P6, s, sw, fc, ec);

%create an all encompassing rectangular prism
s = [14; 14; 12];
sw = .1;
C7 = createRectangularPrism([0, 0, 0]', s, sw, fc, ec, true);

%more cubes
P8 = [.4; 1; .4];
s = 0.5;
sw = .05;
C8 = createCube(P8, s, sw, fc, ec);

P9 = [-.4; 0; -.4];
C9 = createCube(P9, s, sw, fc, ec);

P10 = [-.4; -1; -.4];
C10 = createCube(P10, s, sw, fc, ec);

P11 = [.2; -1; 2];
C11 = createCube(P11, s, sw, fc, ec);

%cube cell array
CArray = cell(2,1);
CArray{1} = C;
CArray{2} = C2;
CArray{3} = C3;
CArray{4} = C4;
CArray{5} = C5;
CArray{6} = C6;
CArray{7} = C7;
CArray{8} = C8;
CArray{9} = C9;
CArray{10} = C10;
CArray{11} = C11;

% createPixelVectors and camera information
f = 500;
width = 640;
height = 480;
sz = [width, height];
px = width/2;
py = height/2;
K = [f, 0, px;
    0, f, py;
    0, 0, 1];

V = createPixelVectors(K,width,height);

%create pixel assignments for lidar points
%build lidar angle array
LidarYawAngles = linspace(LidarFOVWidth/2,-LidarFOVWidth/2,LidarArrayWidth);  %note intentional sign reversal
LidarPitchAngles = linspace(-LidarFOVHeight/2,LidarFOVHeight/2,LidarArrayHeight);
lidarPixelMatches = zeros(LidarArrayHeight,LidarArrayWidth,2);

%create a calibration output
FID = fopen('lidarCalib.csv','w');
fprintf(FID,'%3i, %3i, \n', LidarArrayWidth, LidarArrayHeight);
for ii = 1:LidarArrayWidth
    fprintf(FID,'%8f, ',LidarYawAngles(ii));
end
fprintf(FID,'\n');
for ii = 1:LidarArrayHeight
    fprintf(FID,'%8f, ',LidarPitchAngles(ii));
end
fclose(FID);

for ii = 1:LidarArrayWidth
    for jj = 1:LidarArrayHeight
        %create vector corresponding to these angles
        r = zeros(3,1);
        r(2) = sin(LidarPitchAngles(jj));
        r13 = cos(LidarPitchAngles(jj));
        r(1) = -r13*sin(LidarYawAngles(ii));
        r(3) = r13*cos(LidarYawAngles(ii));

        %map to pixels
        pbar = K*r;
        p = zeros(2,1);
        p(1) = round(pbar(1)/pbar(3));
        p(2) = round(pbar(2)/pbar(3));
        %p(1) = pbar(1)/pbar(3);
        %p(2) = pbar(2)/pbar(3);

        lidarPixelMatches(jj,ii,:) = p;

    end
end

%putput pixels to make sure everything is correct
% figure
% hold on
% scatter(lidarPixelMatches(1,1,1),lidarPixelMatches(1,1,2))
% scatter(lidarPixelMatches(1,end,1),lidarPixelMatches(1,end,2))
% scatter(lidarPixelMatches(end,end,1),lidarPixelMatches(end,end,2))
% scatter(lidarPixelMatches(end,1,1),lidarPixelMatches(end,1,2))
% legend('UL','UR','LR','LL','Location','best')
% set(gca, 'YDir','reverse')

%run through and create an image at each point, always pointing towards the
%center
vz = [0; 0; 1];
vx = [1; 0; 0];
vBMat = [vx'; vz'];
aVec = [1; 1];

% imgFig = figure;
% imgDFig = figure;
% lidarFig = figure;

%generate quaternions
qArray = zeros(N,4);

for ii = 1:N

    %create the quaternion for this location
    imFoc = [0 0 0]'; %point the image is centered on
    vz_I = imFoc - x(:,ii); %camera z-axis in the inertial frame
    vx_I = [vz_I(2); -vz_I(1); 0]; %camera x-axis in the inertial frame

    %normalize vectors
    vx_I = vx_I/norm(vx_I);
    vz_I = vz_I/norm(vz_I);

    %create matrix and solve wahbas problem
    vIMat = [vx_I'; vz_I'];
    RBI = wahbaSolver(aVec,vIMat,vBMat);
    q = dcm2quat(RBI);
    qArray(ii,:) = q;

end

% Generate trajectory truth
se3_tangent = zeros(N,6);
for ii = 1:N
    % Extract rotation matrix
    R = quat2dcm(qArray(ii,:));
    
    % Extract translation
    t = -R*x(:,ii)/scale_slam2truth;

    % Construct 4x4 matrix
    M = [R t;
        0 0 0 1];
    log_M = logm(M);

    % Extract SE3 element
    se3_tangent(ii,1:3) = log_M(1:3,4)';
    se3_tangent(ii,4) = log_M(3,2);
    se3_tangent(ii,5) = log_M(1,3);
    se3_tangent(ii,6) = log_M(2,1);
end

% Get number of rows and columns and add to the file
[r,c] = size(se3_tangent);
toprow = zeros(1,c);
toprow(1) = r;
toprow(2) = c;
se3_tangent = [toprow; se3_tangent];

% Write the matrix
writematrix(se3_tangent,outputfile,"Delimiter",',')

%generate attitude noise if needed
if(addTrajNoise)
    anglenoise = mvnrnd(MuEul,PEul,N);
    quatnoise = angle2quat(anglenoise(:,1),anglenoise(:,2),anglenoise(:,3));
    qArray = quatmultiply(qArray,quatnoise);
end

%control which images are created
if(runTargOnly)
    idxs = targIdx;
else
    idxs = 1:N;
end

%create all the images
% imgDArray = zeros(sz(2),sz(1),length(idxs));
tic

%display progress
fprintf(1, 'Progress: %3d%%',0);

parfor ii = idxs
    if(useMexForImgGen)
        imgRGBD = createImage_mex(CArray, x(:,ii), qArray(ii,:)', V, sz, K);
    else
        imgRGBD = createImage(CArray, x(:,ii), qArray(ii,:)', V, sz, K);
    end

    %extract RGB info
    img = imgRGBD(:,:,1:3);
    imgD = imgRGBD(:,:,4);

    if(createlidar)
        imgLidar = createLidarImage(imgD, lidarPixelMatches);
    end


    %create some noise
    RGBnoise = mvnrnd(MuRGB*ones(sz(1)*sz(2),3), PRGB*eye(3));
    Dnoise = mvnrnd(MuLidar*ones(LidarArrayWidth*LidarArrayHeight,1), PLidar);


    if(addRBGNoise)
        %add noise to img
        for jj = 1:sz(1)
            for kk = 1:sz(2)

                % linear index
                idx = jj + (kk-1)*sz(1);

                %add RBG Noise
                newPixel = squeeze(img(kk,jj,:)) + RGBnoise(idx,:)';

                %constrain RBG values
                newPixel(newPixel > 1) = 1;
                newPixel(newPixel < 0) = 0;

                %reassign
                img(kk,jj,:) = newPixel;

            end
        end
    end

    if(addDepthNoise && createlidar)

        %add noise to imgLidar
        for jj = 1:LidarArrayWidth
            for kk = 1:LidarArrayHeight

                %linear index
                idx = jj + (kk-1)*LidarArrayWidth;

                %depth noise
                imgLidar(kk,jj) = imgLidar(kk,jj) + Dnoise(idx);

            end
        end

    end

    %apply gaussian blur to image
    imgFilt = imgaussfilt(img,GaussBlurFactor);

    %save Depth
    %     if(ii == targKF)

    %     imgDArray(:,:,ii) = imgD;
    %     end

    %display images
    % imgFilt = img;
    % imshow(img);

    % figure(imgFig);
    % imshow(imgFilt);

    % figure(imgDFig)
    % s = surf(imgD);
    % s.EdgeColor = 'interp';
    % view([0 0 -1])

    % figure(lidarFig)
    % s = surf(imgLidar);
    % s.EdgeColor = 'interp';
    % view([0 0 -1])

    %write images
    imwrite(imgFilt,strcat(savepath,'images/cubeCircling',num2str(ii-1,'%04i'),'.jpg'))

    if(createlidar)
        writematrix(imgLidar,strcat(savepath,'lidarImages/cubeCircling',num2str(ii-1,'%04i'),'.csv'),...
            'Delimiter',',');
    end

    %write truth
    if(savetruth)
        fname = strcat(savepath,'truth/truthDepth',string(ii-1),'.csv');
        writematrix(imgD, fname, 'Delimiter',',');
    end

    %display progress
    prog = ii/length(idxs)*100;
    fprintf(1,'\b\b\b\b%3.0f%%',prog);

end
fprintf(1,'\n');
toc

fprintf(1,'Writing out data... ');
%write out truth data
if(~saveAsCsv)
    truth.CArray = CArray;
    truth.traj = x;
    truth.quat = qArray;
    %     truth.depth = imgDArray;
    truth.K = K;
    truth.lidarPixelMatches = lidarPixelMatches;
    save(strcat(savepath,'truth/slamSIM_truth.mat'),...
        'truth','-v7.3');
else
    csvwrite(strcat(savepath,'truth/truthTraj.csv'), x');
    csvwrite(strcat(savepath,'truth/truthQuat.csv'), qArray);
    csvwrite(strcat(savepath,'truth/truthK.csv'), K);
    csvwrite(strcat(savepath,'truth/truthLidarPixelMatchesX.csv'), lidarPixelMatches(:,:,1));
    csvwrite(strcat(savepath,'truth/truthLidarPixelMatchesY.csv'), lidarPixelMatches(:,:,2));

end
fprintf(1,'Done!\n Simulation Complete.\n');

