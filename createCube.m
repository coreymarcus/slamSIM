function C = createCube(P, s, sw, fc, ec, ombrecubes, varombre)
%creates a structure with all the information required to describe a cube
% Inputs
% P = [3x1] position of the center of the cube
% s = 1x1 cube side length
% sw = 1x1 side width, measured from edge to beginning of face
% fc = 6x3 RGB triplets for cube face colors
% ec = 1x3 RGB triplet with the cubes edge color
% ombrecubes = 1x1 bool for using ombre colored sides
% varombre = 1x1 scalar controling variance in ombre colors - higher means
%   more diverse coloring

% assign the initial variables
C.P = P;
C.s = s;
C.sw = sw;
C.ec = ec;
C.ombre = ombrecubes;

%create information for all the faces
C.faces = cell(6,1);
planes = ['x'; 'y'; 'z'];
planes = [planes; planes];
for ii = 1:6
    S.color = fc(ii,:);
    S.plane = planes(ii);

    % Control color
    if(ombrecubes)
        S.colormat = zeros(2,2,3);
        for xx = 1:2
            for yy = 1:2
                % Draw random color
                color = mvnrnd(S.color,varombre*eye(3));
                
                % Saturate
                color(color > 1) = 1;
                color(color < 0) = 0;

                % Assign
                S.colormat(yy,xx,:) = color;
                
            end
        end
    end
    
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

