function I = createImage(C, P, q, V, sz, K)
%creates an RGB image of a cube with a white background given cube
%parameters, camera position and orientation, and camera calibration matrix
% Inputs
% C = [Nx1] cell array of cube structures
% P = [3x1] camera position in inertial frame
% q = [4x1] quaternion rotating from inertial to camera frame, scalar firsts
% V = [height x width x 3] matrix of vectors corresponding to each pixels
% sz = [2x1] = [width, height] image size in pixels
% 
% Outputs
% I = [height x width x 4] RGB+D triplet information

%get number of cubes
Ncubes = length(C);

%initialize image
width = sz(1);
height = sz(2);
Icell = cell(Ncubes,1);

%find the verticies of all the cubes
bounds = ones(Ncubes,4);
for ii = 1:Ncubes
    
    %locate the verticies of the cube in pixel form
    Verts = locateVerticies(C{ii}, P, q, K);
    
    xMin = max([Verts(1) 1]);
    xMax = min([Verts(2) width]);
    yMin = max([Verts(3) 1]);
    yMax = min([Verts(4) height]);
    
    %continue if the box is out of the frame
    if(yMax < 1 || xMax < 1 || xMin >= width || yMin >= height)
        continue;
    else
        bounds(ii,:) = [xMin, xMax, yMin, yMax];
    end
    
end


% cycle throught all the cubes and all the points
Itemplate = ones(height, width, 4);
Itemplate(:,:,4) = zeros(height, width);

for kk = 1:Ncubes
    
    %initialize this cube's image (RGB+D)
    Iloop = Itemplate;
        
    xIdxs = bounds(kk,1):bounds(kk,2);
    yIdxs = bounds(kk,3):bounds(kk,4);
    
    %do a parfor or regular for loop
%     numWorkers = 10;
    
    %check to see if this is actually the all encompassing box
    if(isfield(C{kk},'isAllEncomp') && C{kk}.isAllEncomp)
            xIdxs = 1:width;
            yIdxs = 1:height;
            %numWorkers = 10;
    end
    
%     parfor (ii = xIdxs, numWorkers)
    for ii = xIdxs(1):xIdxs(end)
        
        %temporary row
        tempCol = [ones(length(yIdxs),3) zeros(length(yIdxs),1)];
        
        for jj = yIdxs(1):yIdxs(end)
                         
            %get pixel vector and then rotate it into the inertial frame
            vCam = squeeze(V(jj,ii,:));
            vInert = quatrotateCoder(quatconj(q'),vCam')';
            
            % check intersect
            [inter, D] = checkIntersect(C{kk}, P, vInert);
            
            %intitialize pixel
            Pix = zeros(4,1);
            
            %assign distance
            rBar = D*vCam;
            Pix(4) = rBar(3);
            
            if(inter == 0) 
                %we hit nothing
                continue;
                
            elseif(inter == 7) 
                %we hit an edge
                Pix(1:3) = C{kk}.ec;
                
            else
                %else we hit an face and need to get that edges color
                Pix(1:3) = C{kk}.faces{inter}.color;
                
            end
            
            %build temporary row
            targRow = find(yIdxs == jj);
            targRow = targRow(1);
            tempCol(targRow,1:4) = Pix';
            
        end
        
        Iloop(yIdxs,ii,:) = tempCol;
    end
    
    %add this cube's image to the cell
    Icell{kk} = Iloop;
end

%create final output
I = ones(height, width, 4);
I(:,:,4) = zeros(height, width);

for ii = 1:Ncubes %it might speed things up to integrate this loop with the one above
    
    xIdxs = bounds(ii,1):bounds(ii,2);
    yIdxs = bounds(ii,3):bounds(ii,4);
    
    %check to see if this is actually the all encompassing box
    if(isfield(C{ii},'isAllEncomp') && C{ii}.isAllEncomp)
            xIdxs = 1:width;
            yIdxs = 1:height;
    end
    
    for jj = yIdxs(1):yIdxs(end) %note: a parfor loop is slower here
        for kk = xIdxs(1):xIdxs(end)
            Targ = I(jj,kk,:);
            Pix = squeeze(Icell{ii}(jj,kk,:));
            
            if(Pix(4) == 0)
                %there is nothing here
                continue;
            end
            
            if(Targ(4) == 0)
                %this is the first occupied pixel we've observed
                I(jj,kk,:) = Pix;
                continue;
            end
            
            if(Targ(4) > Pix(4))
                %this pixel is closer than the previously observed one
                I(jj,kk,:) = Pix;
            end
            
        end
    end
end

end



