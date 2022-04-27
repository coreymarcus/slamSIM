% Takes truth data and corrupts it for SLAM ground truth initialization
clear
close all
clc

%% Options

% Data path which contains truth depth
truthdatapath = 'C:\Users\corey\Documents\SharedFolder\SLAMsim_data\RunWithTruth_LowAngleNoise\truth\';

% Distribution for initial corruption
noisedist = 'Gaussian';

% Mean and variance for noise (pre-normalization)
mu = 0;
sigma2 = .1;

% Should we normalize the mean depth to 1?
normalizedata = true;

%% Main

% load the truth data
truthdepth = readmatrix(strcat(truthdatapath,"TruthDepth.csv"));

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

% Normalize the output if needed
if(normalizedata)
    scalefactor = mean(truthdepth,'all');
    noisydepth = noisydepth/scalefactor;
    depthvar = depthvar/scalefactor^2;
end

% Save data
writematrix(noisydepth,[truthdatapath 'DepthInit.csv'],'Delimiter',',');
writematrix(depthvar,[truthdatapath 'DepthVarInit.csv'],'Delimiter',',');
