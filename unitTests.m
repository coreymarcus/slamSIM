%This script tests all my functions and sub functions

clear
close all
clc


%% Create Cube
P = [1; 1; 1];
s = 1;
sw = 0.05;
fc = rand(6,3);
ec = [1 1 1];
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

axis([-4 4 -4 4 -4 4])
grid on
xlabel('x')
ylabel('y')
zlabel('z')

%% Check intersect
x = [0; 1; 1];
v = [1; 0; 0];
checkIntersect(C,x,v)