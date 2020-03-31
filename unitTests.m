%This script tests all my functions and sub functions

clear
close all
clc


%% Create Cube
P = [0; 0; 0];
s = 1;
sw = 0.05;
fc = [1 0 0;
    0 1 0;
    0 0 1;
    1 1 0;
    1 0 1;
    0 1 1];

ec = [.1 .1 .1];
C = createCube(P, s, sw, fc, ec);

%plot all the verticies to see if they're right
figure
hold on
for ii = 1:6
    scatter3(C.faces{ii}.center(1), C.faces{ii}.center(2), ...
        C.faces{ii}.center(3),'x')
    plot3(C.faces{ii}.vertex(1,[1:4 1]), C.faces{ii}.vertex(2,[1:4 1]),...
        C.faces{ii}.vertex(3,[1:4 1]))
end

% axis([-4 4 -4 4 -4 4])
grid on
xlabel('x')
ylabel('y')
zlabel('z')

%% Check intersect
x = [10; 10; 10];
v = [0; 0; 1];
r = vrrotvec(v, P - x);
q = axang2quat(r);
% q = angle2quat(0,pi/2,-pi/4,'ZYX');
v = quatrotate(quatconj(q),v')';
inter = checkIntersect(C,x,v);
scatter3(x(1),x(2),x(3),'o');
quiver3(x(1),x(2),x(3),v(1),v(2),v(3));

%% createPixelVectors
f = 500;
width = 640;
height = 480;
px = width/2;
py = height/2;
K = [f, 0, px;
    0, f, py;
    0, 0, 1];

V = createPixelVectors(K,width,height);
Vunwrap(1,1:width*height) = reshape(V(:,:,1),[1 width*height]); 
Vunwrap(2,1:width*height) = reshape(V(:,:,2),[1 width*height]); 
Vunwrap(3,1:width*height) = reshape(V(:,:,3),[1 width*height]); 

figure
scatter3(Vunwrap(1,1:100:end), Vunwrap(2,1:100:end), Vunwrap(3,1:100:end));
axis([-1 1 -1 1 0 2]);

%% createImages
sz = [width, height];
Ccell = cell(1);
Ccell{1} = C;
imgRGBD = createImage(Ccell, x, q', V, sz, K);
img = imgRGBD(:,:,1:3);
imgblur = imgaussfilt(img,0.5);
imwrite(imgblur,'unitTestImage.png')

% figure
% imshow(imgblur)
figure
imshow(img)