% Takes truth data and corrupts it for SLAM ground truth initialization
clear
close all
clc

%% Options

% Data path which contains truth depth
truthiscsv = true;
% truthdatapath = 'C:\Users\corey\Documents\SharedFolder\SLAMsim_data\RunWithTruth_LowAngleNoise\truth\';
% truthdatapath = 'C:\Users\corey\Documents\SharedFolder\SLAMsim_data\RunWithTruth_LowRot\truth\';
truthdatapath = 'C:\Users\cm58349\Documents\SharedFolder\SLAMsim_data\RunWithTruth_FewCubes\truth\';
% truthdatapath = 'C:\Users\corey\Documents\SharedFolder\EuRoC_data\V2_01_easy\mav0\cam0\truth\';

% Distribution for initial corruption
noisedist = 'Gaussian';

% Mean and variance for noise (pre-normalization)
mu = 0;
sigma2 = .1;

% Should we normalize the mean depth to 1?
normalizedata = true;

%% Main

% load the truth data
if(truthiscsv)
    truthdepth = csvread(strcat(truthdatapath,"TruthDepth.csv"));
    %truthdepth = readmatrix(strcat(truthdatapath,"TruthDepth.csv"));
else
    % This should be the EuRoC truth data
    load(strcat(truthdatapath,'MapTruthData.mat'),'truthimages');
    truthdepth = truthimages;
end

% Size
[rows, cols] = size(truthdepth);

% Create noise
N = rows*cols;

switch noisedist
    case 'Gaussian'
        noise = mvnrnd(mu, sigma2, N);
    otherwise
        error('Invalid noise distribution')
end

% Create output depth and variance
noisydepth = truthdepth + reshape(noise,rows,cols);
depthvar = sigma2*ones(rows,cols);

% Force depth to be positive
noisydepth = abs(noisydepth);

% Normalize the output if needed
if(normalizedata)
    scalefactor = mean(reshape(truthdepth,[],1));
    noisydepth = noisydepth/scalefactor;
    depthvar = depthvar/scalefactor^2;
end

% Save data
% writematrix(noisydepth,[truthdatapath 'DepthInit.csv'],'Delimiter',',');
% writematrix(depthvar,[truthdatapath 'DepthVarInit.csv'],'Delimiter',',');
csvwrite([truthdatapath 'DepthInit.csv'],noisydepth);
csvwrite([truthdatapath 'DepthVarInit.csv'],depthvar);

%% Plotting

figure
mesh(truthdepth,'FaceColor','interp')
title('True Initial Map')

figure
mesh(noisydepth,'FaceColor','interp')
title('Noisy Map Initialization')
