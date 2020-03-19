% Runs a circle around a cube at the origin

clear
close all
clc

%circle parameters
R = 5; %radius
N = 1500; %number of images
inc = 1; %inclination (not a real orbital inclination)
Revs = 2; %number of revolutions around the cube

theta = linspace(0,2*pi*Revs,N);
x = zeros(3,N);
for ii = 1:N
    x(1,ii) = R*cos(theta(ii));
    x(2,ii) = R*sin(theta(ii));
    x(3,ii) = inc*sin(4*theta(ii));
end

figure
scatter3(x(1,:),x(2,:),x(3,:))

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

%run through and create an image at each point, always pointing towards the
%center
v = [0; 0; 1];
figure
tic
for ii = 1:N
   
    %create the quaternion for this location
    r = vrrotvec(v, P - x(:,ii));
    q = axang2quat(r);
    
    img = createImage(C, x(:,ii), q', V, sz);
    imshow(img)
    
    imwrite(img,strcat('images/cubeCircling',num2str(ii,'%04i'),'.jpg'))
    
    disp('Percent Complete:')
    disp(ii/N*100)

end
toc