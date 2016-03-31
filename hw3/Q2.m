% BISWARAJ KAR - CS 5335
% Localize a cylinder in the point cloud. Given a point cloud as input, this
% function should locate the position and orientation, and radius of the
% cylinder.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting cylinder center
%         axis -> 3x1 unit vector pointing along cylinder axis
%         radius -> scalar radius of cylinder
function [center,axis,radius] = Q2(ptCloud)

%Get surface normals based on 10 points
allNormals=pcnormals(ptCloud,20);
%Get total number of points in the Cloud
totalPoints=ptCloud.Count;
%Set the epsilon to search inliners within
epsilon=0.01;
%Initialize the output array
ransac=[];
%Radius Bounds
rLowerLimit=0.05;
rUpperLimit=0.10;
%Max Run Count
maxRunCount=15000;

for runCount = 1:maxRunCount;
    
    %Select 2 random points
    sample1 = randi(totalPoints);
    sample2 = randi(totalPoints);
    
    %Get the coordinates of the 2 random points
    samplePt1 = ptCloud.Location(sample1,:);
    samplePt2 = ptCloud.Location(sample2,:);
    
    %Get the normals at the 2 random points
    samplePt1Normal = allNormals(sample1,:);
    samplePt2Normal = allNormals(sample2,:);
    
    %Get the unit vectors along the normals at the 2 random points
    samplePt1_nor_unit_vec = samplePt1Normal / norm(samplePt1Normal);
    samplePt2_nor_unit_vec = samplePt2Normal / norm(samplePt2Normal);
    
    %Get a random Radius between limits
    randRadius = rLowerLimit + (rUpperLimit - rLowerLimit)*rand;
    
    %Compute centres for the 2 random points
    samplePt1_centre = samplePt1 - samplePt1_nor_unit_vec*randRadius;
    samplePt2_centre = samplePt2 - samplePt2_nor_unit_vec*randRadius;
    
    %Cylinder Calculations:
    %Get the orientation of the axis and the unit vector along the same
    cylinder_orientation = cross(samplePt1Normal,samplePt2Normal);      %Get the cross product of the two points
    cyl_axis_unit_vector = cylinder_orientation / norm(cylinder_orientation);
    
    %Get the projection using (I - aaT)
    projection_matrix=(eye(3) - cyl_axis_unit_vector' * cyl_axis_unit_vector);
    
    %Projection of all points on the plane orthogonal to the axis
    projection_on_plane = projection_matrix * ptCloud.Location';
    
    %Get the first random centre projected onto the plane orthogonal to the axis
    samplePt1_centre_projected = projection_matrix * samplePt1_centre';
    
    %Calculate distance of every projected point in plane from the centre of the
    %circle based on first random point
    dist = sqrt( sum((projection_on_plane - repmat(samplePt1_centre_projected,1,size(projection_on_plane,2))).^2,1) );
    inliers = (dist > (randRadius - epsilon)) & (dist < (randRadius + epsilon));
    
    %Populate RANSAC parameters for current iteration
    ransac(runCount,:) = [samplePt1_centre,randRadius,cyl_axis_unit_vector,sum(inliers)];
    
end
%Get Index where count of inliners is maximum
[~,maxInliner]=max(ransac(:,8));

%Get the center, radius and axis at the above index
center=ransac(maxInliner,1:3)'
radius=ransac(maxInliner,4)
axis=ransac(maxInliner,5:7)'
    
end

