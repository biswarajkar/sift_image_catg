% BISWARAJ KAR - CS 5335
% Localize a sphere in the point cloud. Given a point cloud as input, this
% function should locate the position and radius of a sphere.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting sphere center
%         radius -> scalar radius of sphere
function [center,radius] = Q1(ptCloud)

pcshow(ptCloud);

%Get surface normals based on 10 points
allNormals=pcnormals(ptCloud,20);
%Get total number of points in the Cloud
totalPoints=ptCloud.Count;
%Set the epsilon to search inliners within
epsilon=0.001;
%Initialize the output array
ransac=[];
%Radius Bounds
rLowerLimit=0.05;
rUpperLimit=0.11;
%Max Run Count
maxRunCount=8000;

for runCount=1:1:maxRunCount
    
    nthSample = randi(totalPoints);                     %Select a random point in the point cloud
    samplePt=ptCloud.Location(nthSample,:);             %Get the coordinates of the random point
    samplePtNorm = allNormals(nthSample,:);             %Get the surface normal at the random point
    samplePtNorm_nor_unit_vec = samplePtNorm / norm(samplePtNorm);  %Get the unit vector along the surface normal
    randRadius= (rUpperLimit - rLowerLimit)*rand + rLowerLimit;     %Get a random radius between 0.05 and 0.11
    
    samplePtCentre=samplePt - (samplePtNorm_nor_unit_vec * randRadius);   %Get the coordinates of the centre based on random point
    
    %inliners=[];
    %Calculate distance of each point in cloud from the random centre
%         for n=1:totalPoints
%             dist=norm(samplePtCentre - ptCloud.Location(n,:));
%             if (dist > (randRadius - delta)) && (dist < (randRadius + delta))
%                 isInliner = 1;
%             else
%                 isInliner=0;
%             end
%             inliners(end+1)=isInliner;
%         end

    %Calculate distance of each point in cloud from the random centre
    dist=sqrt(sum((ptCloud.Location - repmat(samplePtCentre,totalPoints,1)).^2,2));
    
    %Get inliners within epsilon distance
    inliners = (dist > (randRadius - epsilon)) & (dist < (randRadius + epsilon));

    %Populate RANSAC parameters for current iteration
    ransac(runCount,:)=[samplePtCentre,randRadius,sum(inliners)];
    
end

%Get Index where count of inliners is maximum
[~,maxInliner]=max(ransac(:,5));

%Get the center and radius at the above index
center=ransac(maxInliner,1:3)
radius=ransac(maxInliner,4)

end
