% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% input: questionNum -> Integer between 1 and 5 that denotes question
%                       number to run.
function hw3(questionNum)

    close all;

    % load point cloud
    load('object3d.mat');
    ptCloudOrig = ptCloud;

    
    % ************* Question 1 *******************
    if questionNum == 1

        % approximately segment object from background
        roi = [-inf,0.5,0.2,0.4,0.1,inf]; % segment sphere
%         roi = [-inf,inf,-inf,inf,-inf,inf]; % no segmentation 
        indices = findPointsInROI(ptCloud,roi);
        ptCloudB = select(ptCloudOrig,indices);
        
        % locate sphere
        [center,radius] = Q1(ptCloudB);

        % display cloud
        figure;
        pcshow(ptCloudOrig);
        hold on;

        % plot sphere
        [X,Y,Z] = sphere;
        X = X * radius + center(1);
        Y = Y * radius + center(2);
        Z = Z * radius + center(3);
        surf(X,Y,Z);
        
    end
    
    % ************* Question 2 *******************
    if questionNum == 2
        
        % approximately segment object from background
        roi = [0.4,0.6,-inf,0.2,0.1,inf]; % cylinder
        indices = findPointsInROI(ptCloud,roi);
        ptCloudB = select(ptCloudOrig,indices);

        % display cloud
        figure;
        pcshow(ptCloudOrig);
        hold on;

        % locate cylinder
        [center,axis,radius] = Q2(ptCloudB);
        axis = axis / norm(axis); % normalize just to be sure...
        
        % plot cylinder
        nn = (eye(3) - axis * axis') * [1;0;0]; % get an arbitrary vector orthogonal to axis
        nn = nn / norm(nn);
        R = [cross(nn,axis) nn axis]; % create rotation matrix
        [X,Y,Z] = cylinder;
        Z=Z*5 - 2.5; % lengthen cylinder
        m = size(X,1)*size(X,2);
        rotXYZ = R * ([reshape(X,1,m); reshape(Y,1,m); reshape(Z,1,m)]*radius) + repmat(center,1,m); % rotate cylinder and add offset
        surf(reshape(rotXYZ(1,:),2,m/2),reshape(rotXYZ(2,:),2,m/2),reshape(rotXYZ(3,:),2,m/2)); % plot it

    end
    
end


