% Runs a circle around a cube
function [] = CubeCircleMHNfcn(N_MC,targIdx)


%% Options

%make sure we get a different seed every time
rng('shuffle');

%circle parameters
rad_circle = 6; %radius
Nrev = 10000; %number of images per revolution
inc = .5; %magnitude of oscilations
Revs = .5; %number of revolutions around the cube
Noscil = 70; %number of oscillations per revolution
N = round(Revs*Nrev);

%savepath for data
savepath = 'data/MonteCarlo/';

%save truth information as csv?
saveAsCsv = true;

% Noise
addTrajNoise = true;
GaussBlurFactor = 1;
P_R = .01; %variance in circle radius
P_inc = .01; %variance in circle height
P_Noscil = .5; %variance in circle rate of oscilation
P_revs = .001; %variance in total number of revolutions
% P_R = 0; %variance in circle radius
% P_inc = 0; %variance in circle height
% P_Noscil = 0; %variance in circle rate of oscilation
% P_revs = 0; %variance in total number of revolutions
MuPos = zeros(3,1); % average se3 position position noise
PPos = .001*eye(3); % se3 tangent position noise covariance
MuEul = zeros(3,1); % average se3 tangent angle noise
PEul = .0001*eye(3); % se3 tangent angle noise covariance

% Scale for output translation
% scale_slam2truth = 5.4988;
scale_slam2truth = 1.0;

%% Main
%path for some useful functions
addpath('submodules/matlabScripts/')

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

%cube cell array rich environment
CArray = cell(1);
CArray{1,1} = C;
CArray{2,1} = C2;
CArray{3,1} = C3;
CArray{4,1} = C4;
CArray{5,1} = C5;
CArray{6,1} = C6;
CArray{7,1} = C7;
CArray{8,1} = C8;
CArray{9,1} = C9;
CArray{10,1} = C10;
CArray{11,1} = C11;

%cube cell array sparse environment
% CArray = cell(1);
% CArray{1,1} = C;
% CArray{2,1} = C2;
% CArray{3,1} = C3;
% CArray{4,1} = C4;

%cube cell array without encompassing cube
% CArray = cell(1);
% CArray{1} = C;
% CArray{2} = C2;
% CArray{3} = C3;
% CArray{4} = C4;
% CArray{5} = C5;
% CArray{6} = C6;
% CArray{7} = C8;
% CArray{8} = C9;
% CArray{9} = C10;
% CArray{10} = C11;

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
    Riter = rad_circle;
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
    qArray_inertial2cam = zeros(N,4);

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
        qArray_inertial2cam(ii,:) = q;

    end

    %generate pose noise if needed
    if(addTrajNoise)

        % Create se3 tangent error
        posnoise = mvnrnd(MuPos',PPos,N);
        anglenoise = mvnrnd(MuEul,PEul,N);

        for ii = 1:N
            % Create 4x4 error matrix
            log_M = zeros(4);
            log_M(1:3,4) = posnoise(ii,:)';
            log_M(1:3,1:3) = CrossProductMat(anglenoise(ii,:)');
    
            M = expm(log_M);

            % Corrupt the position and velocity
            R_actual = quat2rotm(qArray_inertial2cam(ii,:));
            t_actual = -R_actual*x(:,ii);
            M_actual = [R_actual t_actual;
                0 0 0 1];
            M_new = M*M_actual;
            R_new = M_new(1:3,1:3);
            t_new = M_new(1:3,4);
            qArray_inertial2cam(ii,:) = rotm2quat(R_new);
            x(:,ii) = -R_new'*t_new;
        end
    end

    %check to see if the desired directory exists
    itersavepath = strcat(savepath,'run',num2str(MCidx - 1,'%04i'),'/');
    if(~exist(itersavepath,'dir'))
        mkdir(itersavepath);
        mkdir(strcat(itersavepath,'images/'));
        mkdir(strcat(itersavepath,'truth/'));
    end

    % Generate trajectory truth in se3 tangent form
    se3_tangent = zeros(N,6);
    R_inertial2cam0 = quat2dcm(qArray_inertial2cam(1,:));
    for ii = 1:N
        % Extract rotation matrix
        R_inertial2cam = quat2dcm(qArray_inertial2cam(ii,:));
        R_cam02cam = R_inertial2cam*R_inertial2cam0';

        % Extract translation
        t = -R_inertial2cam*(x(:,ii)-x(:,1))/scale_slam2truth;

        % Construct 4x4 matrix
        M = [R_cam02cam t;
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

    %display progress
    fprintf(1, 'Progress: %3d%%',0);

    % for ii = targIdx
    parfor ii = targIdx

        %create image
        imgRGBD = createImage(CArray, x(:,ii), qArray_inertial2cam(ii,:)', V, sz, K);

        %extract RGB info
        img = imgRGBD(:,:,1:3);

        %apply gaussian blur to image
        imgFilt = imgaussfilt(img,GaussBlurFactor);

        %write images
        imwrite(imgFilt,strcat(itersavepath,'images/cubeCircling',num2str(ii-1,'%04i'),'.jpg'))

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
        truth.quat = qArray_inertial2cam;
        truth.K = K;
        truth.se3 = se3_tangent;
        save(strcat(itersavepath,'truth/slamSIM_truth.mat'),...
            'truth','-v7.3');
    else
        csvwrite(strcat(itersavepath,'truth/truthTraj.csv'), x');
        csvwrite(strcat(itersavepath,'truth/truthQuat.csv'), qArray_inertial2cam);
        csvwrite(strcat(itersavepath,'truth/truthK.csv'), K);
        writematrix(se3_tangent,strcat(itersavepath,'truth/TrackingTruth.csv'),"Delimiter",',')
    end
    fprintf('Done! \n')

    fprintf('MC Run %4.0f Complete! \n',MCidx)

end

fprintf(1,'Done!\n Simulation Complete.\n');

end

