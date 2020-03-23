% Strafes a cube at the origin

clear
close all
clc

%image write path
% impath = 'images/cubeStrafe';
impath = '~/Documents/slamSIM/images/cubeStrafe';

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
% IC = [1, 1, -2]'; %starting point
% FC = [-1, -1, -2.5]'; %final point
% d = 10; %distance from cube
% N = 500; %number of images
% x = zeros(3,N);
% x(1,:) = linspace(IC(1),FC(1),N);
% x(2,:) = linspace(IC(2),FC(2),N);
% x(3,:) = linspace(IC(3),FC(3),N);

% Create a multileg strafing path
Nleg = 150; %number of images on each leg
P1 = [.75, .5, -2]';
P2 = [-.7, .45, -2]';
P3 = [-.65, -.4, -2]';
P4 = [1, -.45, -2]';
P = [P1 P2 P3 P4 P1];
Np = size(P,2);
x1 = [];
x2 = [];
x3 = [];
for ii = 1:Np-1
    x1 = [x1 linspace(P(1,ii),P(1,ii+1),Nleg)];
    x2 = [x2 linspace(P(2,ii),P(2,ii+1),Nleg)];
    x3 = [x3 linspace(P(3,ii),P(3,ii+1),Nleg)];
end
x = [x1; x2; x3];
N = (Np-1)*Nleg;

%quaternion
q = [1 0 0 0]';

% run through and create an image at each point
for ii = 1:N
   
    img = createImage(C, x(:,ii), q, V, sz, K);
    imgFilt = imgaussfilt(img,1.2);
    imshow(imgFilt);
    
    imwrite(imgFilt,strcat(impath,num2str(ii,'%04i'),'.jpg'))
    
    disp('Percent Complete:')
    disp(ii/N*100)

end