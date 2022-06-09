% Takes truth data and corrupts it for SLAM ground truth initialization
clear
close all
clc

%% Options

% Data path which contains truth depth
truthiscsv = true;
% truthdatapath = 'C:\Users\corey\Documents\SharedFolder\SLAMsim_data\RunWithTruth_LowAngleNoise\truth\';
% truthdatapath = 'C:\Users\corey\Documents\SharedFolder\SLAMsim_data\RunWithTruth_LowRot\truth\';
% truthdatapath = 'C:\Users\cm58349\Documents\SharedFolder\SLAMsim_data\RunWithTruth_FewCubes\truth\';
% truthdatapath = 'C:\Users\corey\Documents\SharedFolder\SLAMsim_data\RunWithTruth_FewCubes\truth\';
% truthdatapath = 'C:\Users\corey\Documents\SharedFolder\SLAMsim_data\RunWithTruth_ThinLines\truth\';
% truthdatapath = 'C:\Users\corey\Documents\SharedFolder\SLAMsim_data\Ombre\truth\';
truthdatapath = 'C:\Users\corey\Documents\SharedFolder\SLAMsim_data\Plunge\truth\';
% truthdatapath = 'C:\Users\corey\Documents\SharedFolder\EuRoC_data\V2_01_easy\mav0\cam0\truth\';

% Add path for gammaSLAM
addpath("../GammaSLAM/BasicTesting/")

% Distribution for initial corruption
% noisedist = 'Gaussian';
noisedist = 'MHN';

% Mean and variance for noise (pre-normalization)
mu = 0;
sigma2 = .1;

% Should we normalize the mean depth to 1?
normalizedata = true;

% Should we eliminate knowledge of some segment?
eliminatesegment = false;
eliminatetarget = [247 394 168 318];
newdepth = 12;

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

% Eliminate some portion of the depthmap if needed
truthdeptheliminate = truthdepth;
if(eliminatesegment)
    truthdeptheliminate(eliminatetarget(3):eliminatetarget(4),...
        eliminatetarget(1):eliminatetarget(2)) = newdepth;
end

switch noisedist
    case 'Gaussian'

        % Create noise
        noise = mvnrnd(mu, sigma2, N);

        % Create output depth and variance
        noisydepth = truthdeptheliminate + reshape(noise,rows,cols);
        depthvar = sigma2*ones(rows,cols);

        % Force depth to be positive
        noisydepth = abs(noisydepth);

        % Normalize the output if needed
        if(normalizedata)
            scalefactor = mean(reshape(truthdeptheliminate,[],1));
            noisydepth = noisydepth/scalefactor;
            depthvar = depthvar/scalefactor^2;
        end

    case 'MHN'

        % Reshape truth depth to vector
        truthdepthvec = reshape(truthdeptheliminate,N,1);
        noisydepthvec = zeros(N,1);

        % Output depth variance
        depthvar = sigma2*ones(rows,cols);

        % Sample noise
        alpha = 1;
        beta = 1/(2*sigma2);
        for ii = 1:N
            mu_iter = truthdepthvec(ii);
            gamma_iter = mu_iter/sigma2;
            noisydepthvec(ii) = SampleMHN(alpha,beta,gamma_iter,1);
        end

        % Normalize output if needed
        if(normalizedata)
            scalefactor = mean(reshape(truthdeptheliminate,[],1));
            noisydepthvec = noisydepthvec/scalefactor;
            depthvar = depthvar/scalefactor^2;
        end

        % Reshape into vector
        noisydepth = reshape(noisydepthvec,rows,cols);

    otherwise
        error('Invalid noise distribution')
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

if(eliminatesegment)
    figure
    mesh(truthdeptheliminate,'FaceColor','interp')
    title('True Initial Map With Eliminated Area')
end

figure
mesh(noisydepth,'FaceColor','interp')
title('Noisy Map Initialization')
