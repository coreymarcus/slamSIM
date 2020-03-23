% Strafes a cube at the origin

clear
close all
clc

% Create the cube
P = [0; 0; 0];
s = 0.5;
sw = 0.05;
fc = [1 0 0;
    0 1 0;
    0 0 1;
    1 1 0;
    1 0 1;
    0 1 1];

ec = [1 1 1];
C = createCube(P, s, sw, fc, ec);

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

% Create strafeing path
IC = [-4, -4, -10]'; %starting point
FC = [2, 2, -2]'; %final point
d = 10; %distance from cube
N = 10; %number of images
x = zeros(3,N);
x(1,:) = linspace(IC(1),FC(1),N);
x(2,:) = linspace(IC(2),FC(2),N);
x(3,:) = linspace(IC(3),FC(3),N);

%quaternion
q = [1 0 0 0]';

% run through and create an image at each point
for ii = 1:N
   
    img = createImage(C, x(:,ii), q, V, sz, K);
    imshow(imgaussfilt(img,.75));
    
    imwrite(img,strcat('images/cubeStrafe',num2str(ii,'%04i'),'.jpg'))
    
    disp('Percent Complete:')
    disp(ii/N*100)

end