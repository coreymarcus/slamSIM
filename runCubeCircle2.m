% Runs a circle around a cube

clear
close all
clc

%circle parameters
R = 6; %radius
N = 1000; %number of images
inc = 0; %inclination (not a real orbital inclination)
Revs = 2; %number of revolutions around the cube

theta = linspace(0,2*pi*Revs,N);
x = zeros(3,N);
for ii = 1:N
    x(1,ii) = R*cos(theta(ii));
    x(2,ii) = R*sin(theta(ii));
    % x(3,ii) = inc*sin(4*theta(ii));
    x(3,ii) = inc;
end

figure
scatter3(x(1,:),x(2,:),x(3,:))

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
P2 = [0; 0; -3];
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

%cube cell array
CArray = cell(2,1);
CArray{1} = C;
CArray{2} = C2;
CArray{3} = C3;
CArray{4} = C4;
CArray{5} = C5;
CArray{6} = C6;
CArray{7} = C7;

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
v = [0; 0; 1];
figure
tic
for ii = 1:N
   
    %create the quaternion for this location
    imFoc = [0 0 0]'; %point the image is centered on
    vec2cent = imFoc - x(:,ii);
    r = vrrotvec(v, imFoc - x(:,ii));
%     theta = atan2(vec2cent(2),vec2cent(1));
%     q = angle2quat(theta,-pi/2,0,'ZXY');
%     q = quatconj(q);
    q = axang2quat(r);
    
    imgRGBD = createImage(CArray, x(:,ii), q', V, sz, K);
    
    %extract RGB info
    img = imgRGBD(:,:,1:3);
    
    %filter and display
    imgFilt = imgaussfilt(img,1.2);
    imshow(imgFilt);
    
    imshow(imgFilt);
    
    imwrite(imgFilt,strcat('images/cubeCircling',num2str(ii,'%04i'),'.jpg'))
    
    disp('Percent Complete:')
    disp(ii/N*100)

end
toc