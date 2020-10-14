% Runs a circle around a cube
function [] = runCubeCircle2fcn(N_MC,targIdx)


%% Options

%make sure we get the same seed for with and without lidar
rng(5);

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
useMexForImgGen = true;

%target KF, zero index
targKF = 25;

%savepath for data
% savepath = 'C:\Users\corey\Documents\SharedFolder\truth';
savepath = 'data/';

%save truth information as csv?
saveAsCsv = true;

% Noise
addDepthNoise = true;
addTrajNoise = true;
MuLidar = 0; %average lidar depth noise
PLidar = .01^2; % lidar depth covariance, m^2
GaussBlurFactor = 1;
P_R = .1; %variance in circle radius
P_inc = .1; %variance in circle height
P_Noscil = 5; %variance in circle rate of oscilation
P_revs = .01; %variance in total number of revolutions


%% Main
%path for wahba solver
addpath('../matlabScripts/')

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
        
        lidarPixelMatches(jj,ii,:) = p;
        
    end
end

%run through and create an image at each point, always pointing towards the
%center
vz = [0; 0; 1];
vx = [1; 0; 0];
vBMat = [vx'; vz'];
aVec = [1; 1];

%loop through all MC
for MCidx = 1:N_MC
    
    %create iteration specific variables for the trajectory parameters,
    %these may be corrupted by noise
    Riter = R;
    inciter = inc;
    Nosciliter = Noscil;
    Revsiter = Revs;
    
    %generate attitude and position noise if needed
    if(addTrajNoise)
        Riter = Riter + mvnrnd(0,P_R);
        inciter = inciter + mvnrnd(0,P_inc);
        Nosciliter = Nosciliter + mvnrnd(0,P_Noscil);
        Revsiter = Revsiter + mvnrnd(0,P_revs);
    end
    
    %Create the positioning for this system
    theta = linspace(0,2*pi*Revsiter,N);
    phi = inciter*sin(Nosciliter*theta);
    x = zeros(3,N);
    for ii = 1:N
        x(1,ii) = Riter*cos(theta(ii))*cos(phi(ii));
        x(2,ii) = Riter*sin(theta(ii))*cos(phi(ii));
        x(3,ii) = Riter*sin(phi(ii));
    end
    
    %generate quaternions
    qArray = zeros(N,4);
    
    % Create the attitude for the system
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
    
    %check to see if the desired directory exists
    itersavepath = strcat(savepath,'run',num2str(MCidx - 1,'%04i'),'/');
    if(~exist(itersavepath,'dir'))
        mkdir(itersavepath);
        mkdir(strcat(itersavepath,'images/'));
        mkdir(strcat(itersavepath,'lidarImages/'));
        mkdir(strcat(itersavepath,'truth/'));
    end
    
    %display progress
    fprintf(1, 'Progress: %3d%%',0);
    
    parfor ii = targIdx
        
        %create image
        if(useMexForImgGen)
            imgRGBD = createImage_mex(CArray, x(:,ii), qArray(ii,:)', V, sz, K);
        else
            imgRGBD = createImage(CArray, x(:,ii), qArray(ii,:)', V, sz, K);
        end
        
        %extract RGB info
        img = imgRGBD(:,:,1:3);
        imgD = imgRGBD(:,:,4);
        imgLidar = createLidarImage(imgD, lidarPixelMatches);
        
        %create some depth noise
        Dnoise = mvnrnd(MuLidar*ones(LidarArrayWidth*LidarArrayHeight,1), PLidar);
        
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
        
        %write images
        imwrite(imgFilt,strcat(itersavepath,'images/cubeCircling',num2str(ii-1,'%04i'),'.jpg'))
        dlmwrite(strcat(itersavepath,'lidarImages/cubeCircling',num2str(ii-1,'%04i'),'.csv'),imgLidar,...
            'precision','%.4f')
        
        %write truth if greater than whatever our target is
        if((ii-1) >= targKF)
            fname = strcat(itersavepath,'truth/truthDepth',string(ii-1),'.csv');
            csvwrite(fname, imgD);
        end
        
        %display progress
        prog = ii/N*100;
        fprintf(1,'\b\b\b\b%3.0f%%',prog);
        
    end
    fprintf(1,'\n');
    
    fprintf(1,'Writing out data... ');
    %write out truth data
    if(~saveAsCsv)
        truth.CArray = CArray;
        truth.traj = x;
        truth.quat = qArray;
        truth.K = K;
        truth.lidarPixelMatches = lidarPixelMatches;
        save(strcat(itersavepath,'truth/slamSIM_truth.mat'),...
            'truth','-v7.3');
    else
        csvwrite(strcat(itersavepath,'truth/truthTraj.csv'), x');
        csvwrite(strcat(itersavepath,'truth/truthQuat.csv'), qArray);
        csvwrite(strcat(itersavepath,'truth/truthK.csv'), K);
        csvwrite(strcat(itersavepath,'truth/truthLidarPixelMatchesX.csv'), lidarPixelMatches(:,:,1));
        csvwrite(strcat(itersavepath,'truth/truthLidarPixelMatchesY.csv'), lidarPixelMatches(:,:,2));
        
    end
    fprintf('Done! \n')
    
    fprintf('MC Run %4.0f Complete! \n',MCidx)
    
end

fprintf(1,'Done!\n Simulation Complete.\n');

end

