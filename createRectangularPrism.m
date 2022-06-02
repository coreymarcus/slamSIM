function C = createRectangularPrism(P, s, sw, fc, ec, isAllEncomp)
%creates a structure with all the information required to describe a
%rectangular prism
% Inputs
% P = [3x1] position of the center of the prism
% s = [3x1] prism side length, [x y z]
% sw = 1x1 side width, measured from edge to beginning of face
% fc = 6x3 RGB triplets for prism face colors
% ec = 1x3 RGB triplet with the prism edge color
% isAllEncomp = 1x1 bool controlling if the prism ecompasses the entire
%   simulation

% assign the initial variables
C.P = P;
C.s = s;
C.sw = sw;
C.ec = ec;
C.isAllEncomp = isAllEncomp;

%create information for all the faces
C.faces = cell(6,1);
planes = ['x'; 'y'; 'z'];
planes = [planes; planes];
C.ombre = false;
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
        S.center = P + a*[.5*s(1); 0; 0];
        S.vertex = zeros(3,4);
        S.vertex(:,1) = S.center + [0; -0.5*s(2); -0.5*s(3)];
        S.vertex(:,2) = S.center + [0; 0.5*s(2); -0.5*s(3)];
        S.vertex(:,3) = S.center + [0; 0.5*s(2); 0.5*s(3)];
        S.vertex(:,4) = S.center + [0; -0.5*s(2); 0.5*s(3)];
        
    elseif(strcmp(S.plane,'y'))
        
        S.center = P + a*[0; .5*s(2); 0];
        S.vertex = zeros(3,4);
        S.vertex(:,1) = S.center + [-0.5*s(1); 0; -0.5*s(3)];
        S.vertex(:,2) = S.center + [0.5*s(1); 0; -0.5*s(3)];
        S.vertex(:,3) = S.center + [0.5*s(1); 0; 0.5*s(3)];
        S.vertex(:,4) = S.center + [-0.5*s(1); 0; 0.5*s(3)];
        
    elseif(strcmp(S.plane,'z'))
        
        S.center = P + a*[0; 0; .5*s(3)];
        S.vertex = zeros(3,4);
        S.vertex(:,1) = S.center + [-0.5*s(1); -0.5*s(2); 0];
        S.vertex(:,2) = S.center + [0.5*s(1); -0.5*s(2); 0];
        S.vertex(:,3) = S.center + [0.5*s(1); 0.5*s(2); 0];
        S.vertex(:,4) = S.center + [-0.5*s(1); 0.5*s(2); 0];
        
    end
    
    C.faces{ii} = S;
    
end



end

