function C = createCube(P, s, sw, fc, ec)
%creates a structure with all the information required to describe a cube
% Inputs
% P = [3x1] position of the center of the cube
% s = 1x1 cube side length
% sw = 1x1 side width, measured from edge to beginning of face
% fc = 6x3 RGB triplets for cube face colors
% ec = 1x3 RGB triplet with the cubes edge color

% assign the initial variables
C.P = P;
C.s = s;
C.sw = sw;
C.ec = ec;

%create information for all the faces
C.faces = cell(6,1);
planes = ['x'; 'y'; 'z'];
planes = [planes; planes];
for ii = 1:6
    S.color = fc(ii,:);
    S.plane = planes(ii);
    
    %switching on the planes
    if(ii < 4)
        a = 1;
    else
        a = -1;
    end
    
    if(strcmp(S.plane,'x')) 
        S.center = P + a*[.5*s; 0; 0];
        S.vertex = zeros(3,4);
        S.vertex(:,1) = S.center + [0; -0.5*s; -0.5*s];
        S.vertex(:,2) = S.center + [0; 0.5*s; -0.5*s];
        S.vertex(:,3) = S.center + [0; 0.5*s; 0.5*s];
        S.vertex(:,4) = S.center + [0; -0.5*s; 0.5*s];
        
    elseif(strcmp(S.plane,'y'))
        
        S.center = P + a*[0; .5*s; 0];
        S.vertex = zeros(3,4);
        S.vertex(:,1) = S.center + [-0.5*s; 0; -0.5*s];
        S.vertex(:,2) = S.center + [0.5*s; 0; -0.5*s];
        S.vertex(:,3) = S.center + [0.5*s; 0; 0.5*s];
        S.vertex(:,4) = S.center + [-0.5*s; 0; 0.5*s];
        
    elseif(strcmp(S.plane,'z'))
        
        S.center = P + a*[0; 0; .5*s];
        S.vertex = zeros(3,4);
        S.vertex(:,1) = S.center + [-0.5*s; -0.5*s; 0];
        S.vertex(:,2) = S.center + [0.5*s; -0.5*s; 0];
        S.vertex(:,3) = S.center + [0.5*s; 0.5*s; 0];
        S.vertex(:,4) = S.center + [-0.5*s; 0.5*s; 0];
        
    end
    
    C.faces{ii} = S;
    
end



end

