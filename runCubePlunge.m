% Runs a circle around a cube

clear
close all


%% Options

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
Revs = .01; %number of revolutions around the cube before starting plunge
Noscil = 70; %number of oscillations per revolution
PlungeDepth = 3; %distance from center when plunge ends

%control if we used compiled code for image generation
useMexForImgGen = false;

%true depth data is massive, run this if you only want to create and save
%it at the target index
targIdx = [1:100]; %set of frames we'd like to run (1-idx, not 0-idx)
targKF = targIdx(1); %the frame where dense depth data for each image pixel will be saved
runTargOnly = false;

%savepath for data
% savepath = 'C:\Users\corey\Documents\SharedFolder\truth';
savepath = 'truth';

%save truth information as csv?
saveAsCsv = true;

% Noise
addRBGNoise = false;
addDepthNoise = true;
MuLidar = 0; %average lidar depth noise
PLidar = .01^2; % lidar depth covariance, m^2
MuRGB = 0; % average RGB noise
PRGB = .01; % RGB noise covariance
GaussBlurFactor = 3;

% Parallel Pool
poolnum = 8; %number of cores used

%% Main

%path for wahba solver
addpath('../matlabScripts/')

%generate trajectory
Nperd = Nrev/(2*pi*R); % number of images per m travelled
Nrev = round(Revs*Nrev);
Nlin = round(PlungeDepth*Nperd);
N = Nlin + Nrev;

theta = linspace(0,2*pi*Revs,Nrev);
phi = inc*sin(Noscil*theta);
x = zeros(3,N);
for ii = 1:Nrev
    x(1,ii) = R*cos(theta(ii))*cos(phi(ii));
    x(2,ii) = R*sin(theta(ii))*cos(phi(ii));
    x(3,ii) = R*sin(phi(ii));
end

%linear portion of trajectory
revEnd = x(:,Nrev);
linEnd = PlungeDepth*revEnd/norm(revEnd);
x(1,Nrev+1:end) = linspace(revEnd(1),linEnd(1),Nlin);
x(2,Nrev+1:end) = linspace(revEnd(2),linEnd(2),Nlin);
x(3,Nrev+1:end) = linspace(revEnd(3),linEnd(3),Nlin);

figure
scatter3(x(1,:),x(2,:),x(3,:))
axis equal

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
figure
hold on
scatter(lidarPixelMatches(1,1,1),lidarPixelMatches(1,1,2))
scatter(lidarPixelMatches(1,end,1),lidarPixelMatches(1,end,2))
scatter(lidarPixelMatches(end,end,1),lidarPixelMatches(end,end,2))
scatter(lidarPixelMatches(end,1,1),lidarPixelMatches(end,1,2))
legend('UL','UR','LR','LL','Location','best')
set(gca, 'YDir','reverse')

%run through and create an image at each point, always pointing towards the
%center
vz = [0; 0; 1];
vx = [1; 0; 0];
vBMat = [vx'; vz'];
aVec = [1; 1];

imgFig = figure;
imgDFig = figure;
lidarFig = figure;

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

%control which images are created
if(runTargOnly)
    idxs = targIdx;
else
    idxs = 1:N;
end

%create all the images
imgDArray = zeros(sz(2),sz(1),length(idxs));

% Parallel Pool
p = gcp('nocreate');
if(isempty(p))
    parpool(poolnum)
end

tic

parfor ii = idxs
%     tic
    if(useMexForImgGen)
        imgRGBD = createImage_mex(CArray, x(:,ii), qArray(ii,:)', V, sz, K);
    else
        imgRGBD = createImage(CArray, x(:,ii), qArray(ii,:)', V, sz, K);
    end
        
%     toc
    
    %extract RGB info
    img = imgRGBD(:,:,1:3);
    imgD = imgRGBD(:,:,4);
    imgLidar = createLidarImage(imgD, lidarPixelMatches);
    

    
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

    if(addDepthNoise)
        
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
    imgDArray(:,:,ii) = imgD;
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
    
    imwrite(imgFilt,strcat('images/cubeCircling',num2str(ii,'%04i'),'.jpg'))
    dlmwrite(strcat('lidarImages/cubeCircling',num2str(ii,'%04i'),'.csv'),imgLidar,...
        'precision','%.4f')
    
    disp('Percent Complete:')
    disp(ii/length(idxs)*100)
    
end
toc

%write out truth data
if(~saveAsCsv)
    truth.CArray = CArray;
    truth.traj = x;
    truth.quat = qArray;
%     truth.depth = imgDArray;
    truth.K = K;
    truth.lidarPixelMatches = lidarPixelMatches;
    save(strcat(savepath,'slamSIM_truth.mat'),...
        'truth','-v7.3');
else
    csvwrite(strcat(savepath,'/truthTraj.csv'), x');
    csvwrite(strcat(savepath,'/truthQuat.csv'), qArray);
    csvwrite(strcat(savepath,'/truthK.csv'), K);
    csvwrite(strcat(savepath,'/truthLidarPixelMatchesX.csv'), lidarPixelMatches(:,:,1));
    csvwrite(strcat(savepath,'/truthLidarPixelMatchesY.csv'), lidarPixelMatches(:,:,2));
    
    %SAVING TRUTH DEPTH IN ZERO INDEX FILE NAME
    for ii = idxs
        fname = strcat(savepath,'/truthDepth',string(ii-1),'.csv');
        csvwrite(fname, imgDArray(:,:,ii == idxs));
        
    end
end
    
