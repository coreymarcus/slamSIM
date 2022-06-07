% Runs a circle around a cube
function [] = CubePlungeMHNfcn(varargin)

%% Parse inputs
if(isempty(varargin))
    N_MC = 1;
    targIdx = 1;
    seed = 1;
else
    N_MC = varargin{1};
    targIdx = varargin{2};
    seed = varargin{3};
end

%% Options

%set random number generator seed
rng(seed);

%plunge parameters
fcam = 20; % Camera frame rate
x0 = [6 0 0]'; % Initial position [m]
v0 = [-1 .1 1]'; % Initial velocity [m/s]
dt = 1/fcam; % Time step
N = length(targIdx); % Number of frames to create

%savepath for data
savepath = 'data/MonteCarlo/';

%save truth trajectory information as csv?
savetruthtrajascsv = true;

% Save truth depth information as csv?
savetruthdepth = true;
truthdepthtargs = 1;

% Should we zero out rotation
zerorotation = false;

% Should the camera make some tilting motions?
usetiltingmotion = true;

% Upscale the image?
useupscale = false;
upscale = 2; % [width, height] -> [upscale*width, upscale*height]

% Noise
addTrajNoise = true;
GaussBlurFactor = 0.0;
MuPos = zeros(3,1); % average se3 position position noise
PPos = 0*eye(3); % se3 tangent position noise covariance
MuEul = zeros(3,1); % average se3 tangent angle noise
PEul = 0*eye(3); % se3 tangent angle noise covariance

% Scale for output translation
scale_slam2truth = 6.4;
% scale_slam2truth = 1.0;

%% Main
%path for some useful functions
addpath('submodules/matlabScripts/')

% Create the cube
P = [1; 0; 0];
s = 0.5;
ombrecubes = false;
varombre = 0.25;

% sw = 0.02;
sw = 0.0;
fc = [1 0 0;
    0 1 0;
    0 0 1;
    1 1 0;
    1 0 1;
    0 1 1];

ec = [.1 .1 .1];
C1 = createCube(P, s, sw, fc, ec, ombrecubes, varombre);

%create more cubes
P2 = [0; 0; -2];
C2 = createCube(P2, s, sw, fc, ec, ombrecubes, varombre);

P3 = [1; -1; 0];
C3 = createCube(P3, s, sw, fc, ec, ombrecubes, varombre);

P4 = [0; 2; .5];
C4 = createCube(P4, s, sw, fc, ec, ombrecubes, varombre);

P5 = [-1; 0; 0];
C5 = createCube(P5, s, sw, fc, ec, ombrecubes, varombre);

P6 = [.4; -1; .4];
C6 = createCube(P6, s, sw, fc, ec, ombrecubes, varombre);

%create an all encompassing rectangular prism
s = [14; 14; 12];
% C7 = createRectangularPrism([0, 0, 0]', s, sw, fc, ec, true);
C7 = createCube([0, 0, 0]', 12, sw, fc, ec, ombrecubes, varombre);
C7.isAllEncomp = true;

%more cubes
P8 = [.4; 1; .4];
s = 0.5;
C8 = createCube(P8, s, sw, fc, ec, ombrecubes, varombre);

P9 = [-.4; 0; -.4];
C9 = createCube(P9, s, sw, fc, ec, ombrecubes, varombre);

P10 = [-.4; -1; -.4];
C10 = createCube(P10, s, sw, fc, ec, ombrecubes, varombre);

P11 = [.2; -1; 2];
C11 = createCube(P11, s, sw, fc, ec, ombrecubes, varombre);

% MORE
P12 = [0; 3; 1.5];
C12 = createCube(P12, s, sw, fc, ec, ombrecubes, varombre);

P13 = [0; 0; 1.5];
C13 = createCube(P13, s, sw, fc, ec, ombrecubes, varombre);

P14 = [1; -1; 2];
C14 = createCube(P14, s, sw, fc, ec, ombrecubes, varombre);

P15 = [1; -2; -1];
C15 = createCube(P15, s, sw, fc, ec, ombrecubes, varombre);

P16 = [0; 2; 2];
C16 = createCube(P16, s, sw, fc, ec, ombrecubes, varombre);

P17 = [-5; 2; -2];
C17 = createCube(P17, s, sw, fc, ec, ombrecubes, varombre);

P18 = [-5; -4; 1];
C18 = createCube(P18, s, sw, fc, ec, ombrecubes, varombre);

P19 = [4 0 0]';
C19 = createCube(P19, s, sw, fc, ec, ombrecubes, varombre);

%cube cell array rich environment
% CArray = cell(1);
% CArray{1,1} = C;
% CArray{2,1} = C2;
% CArray{3,1} = C3;
% CArray{4,1} = C4;
% CArray{5,1} = C5;
% CArray{6,1} = C6;
% CArray{7,1} = C7;
% CArray{8,1} = C8;
% CArray{9,1} = C9;
% CArray{10,1} = C10;
% CArray{11,1} = C11;

%cube cell array sparse environment
CArray = cell(1);
CArray{1,1} = C1;
CArray{2,1} = C2;
CArray{3,1} = C3;
CArray{4,1} = C4;
CArray{5,1} = C7;
CArray{6,1} = C12;
CArray{7,1} = C13;
CArray{8,1} = C14;
CArray{9,1} = C15;
CArray{10,1} = C16;
CArray{11,1} = C17;
CArray{12,1} = C18;
CArray{13,1} = C19;

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
% width = 640/4;
% height = 480/4;
sz = [width, height];
px = width/2;
py = height/2;
K = [f, 0, px;
    0, f, py;
    0, 0, 1];
Korig = K;

if(useupscale)
    K(1:2,:) = upscale*K(1:2,:);
    sz = upscale*sz;
end

V = createPixelVectors(K,sz(1),sz(2));

%loop through all MC
for MCidx = 1:N_MC

    %Create the positioning for this system
    x = zeros(3,N);
    x(:,1) = x0;
    for ii = 2:N
        x(:,ii) = x(:,ii-1) + dt*v0;
    end

    figure, scatter3(x(1,targIdx),x(2,targIdx),x(3,targIdx))
    axis equal

    %generate quaternions
    qArray_inertial2cam = zeros(N,4);

    % Create the attitude for the system
    for ii = 1:N

        % Initialize some vectors for whabaSolver
        vz = [0; 0; 1];
        vx = [1; 0; 0];
        vBMat = [vx'; vz'];
        aVec = [1; 1];

        %create the quaternion for this location
        if(usetiltingmotion)
            imFoc = [0 0 0]'; %point the image is centered on
        else
            imFoc = [0 0 x(3,ii)]'; %point the image is centered on
        end
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

        % Zero rotation
        if(zerorotation)
            qArray_inertial2cam(ii,:) = qArray_inertial2cam(1,:);
        end

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
    se3_tangent = zeros(N+1,6);
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
        se3_tangent(ii+1,1:3) = log_M(1:3,4)';
        se3_tangent(ii+1,4) = log_M(3,2);
        se3_tangent(ii+1,5) = log_M(1,3);
        se3_tangent(ii+1,6) = log_M(2,1);
    end

    % Add number of rows and columns to the file
    se3_tangent(1,1) = N;
    se3_tangent(1,2) = 6;

%     for ii = targIdx
    parfor ii = targIdx

        %create image
        imgRGBD_upscale = createImage(CArray, x(:,ii), qArray_inertial2cam(ii,:)', V, sz, K);

        % Resize the RGB image
        if(useupscale)
            imgRGBD = zeros(height,width,4);
            imgRGBD(:,:,1:3) = imresize(imgRGBD_upscale(:,:,1:3),[height width],'Method','bilinear');
            imgRGBD(:,:,4) = imresize(imgRGBD_upscale(:,:,4),[height width],'Method','bilinear');
        else
    	    imgRGBD = imgRGBD_upscale;
    	end

        %extract RGB info
        img = imgRGBD(:,:,1:3);

        %apply gaussian blur to image
        if(GaussBlurFactor > 0)
            imgFilt = imgaussfilt(img,GaussBlurFactor);
        else
            imgFilt = img;
        end

        %write images
        imwrite(imgFilt,strcat(itersavepath,'images/cubeCircling',num2str(ii-1,'%04i'),'.jpg'))

        % Consider saving the truth depth
        if(savetruthdepth && ismember(ii,truthdepthtargs))
            writematrix(imgRGBD(:,:,4),...
                strcat(itersavepath, 'truth/TruthDepth.csv'),...
                "Delimiter",",");
        end

        %display progress
        prog = ii/N*100;
        disp(prog)

    end

    fprintf(1,'Writing out data... ');
    %write out truth data
    if(~savetruthtrajascsv)
        truth.CArray = CArray;
        truth.traj = x;
        truth.quat = qArray_inertial2cam;
        truth.K = Korig;
        truth.se3 = se3_tangent;
        save(strcat(itersavepath,'truth/slamSIM_truth.mat'),...
            'truth','-v7.3');
    else
        writematrix(x',...
            strcat(itersavepath,'truth/truthTraj.csv'),...
            "Delimiter",",");
        writematrix(qArray_inertial2cam,...
            strcat(itersavepath,'truth/truthQuat.csv'),...
            "Delimiter",",");
        writematrix(Korig,...
            strcat(itersavepath,'truth/truthK.csv'),...
            "Delimiter",",");
        writematrix(se3_tangent,...
            strcat(itersavepath,'truth/TrackingTruth.csv'),...
            "Delimiter",',')
    end
    fprintf('Done! \n')

    fprintf('MC Run %4.0f Complete! \n',MCidx)

end

fprintf(1,'Done!\n Simulation Complete.\n');

end

