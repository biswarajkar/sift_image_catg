% Calculate a path from qStart to xGoal
% input: qStart -> 1x4 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end effector
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xn vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You may output any number of
%                    milestones. The first milestone should be qStart. The
%                    last milestone should place the end effector at xGoal.
function qMilestones = Q1(rob,sphereCenter,sphereRadius,qStart,xGoal)

%Create a 3-link robot for computing position of end of L1
L1(1) = Link([0 0 0 1.571]);
L1(2) = Link([0 0 0 -1.571]);
L1(3) = Link([0 0.4318 0 -1.571]);
rob3Link = SerialLink(L1,'name','robot');

%Initialize the current angles and positions
posGoal=xGoal;
posInit= rob.fkine(qStart);
posInit=posInit(1:3,4);     %Extract the co-ordinates of the starting position of end-effector
posErrFromGoal= (posGoal-posInit);   %Compute the initial error between the start and end postions of the end-effector

errLimit=0.0;                 %Define a small error limit for distace to goal
stepSize=0.5;               %Define a step size (max distance between two configurations)
runLimit=1000;

qTree=[qStart,0];           %Set the first element of the tree
posErrqNearGoal=inf;        %Set the distance of random config. from goal as infinity

for counter=1:1:runLimit

    %Generate a random configuration
    qRand=(-pi + 2*pi*rand(1,4));
    
    %Find the closest configuration in the existing tree to the
    %random configuration
    qCurrOnTree= findNearest(qRand,qTree);
    qCurr=qCurrOnTree(1,1:4);
    
    %Generate a configuration at 'stepSize' distance from the tree
    %towards the random configuration
    qNear=qCurr+(stepSize*((qRand-qCurr)/norm(qRand-qCurr)));
    
    %Check if the new configuration collides with the obstacle
    [collision] = checkCollision(rob,rob3Link,qCurr,qNear,sphereCenter,sphereRadius);
    posErrqNearGoal=collision.EFPos-posGoal;
    
    %If there is no collision of the new configuration, add it to
    %the tree
    if (collision.colsn==0) && (norm(posErrqNearGoal)<norm(posErrFromGoal))
        posErrFromGoal=posErrqNearGoal;
        qNearWithParent=[qNear qCurrOnTree(1,5)];
        qTree=[qTree;qNearWithParent];
        
        if norm(posErrFromGoal)<=errLimit
            break;
        end
    end
end

%Display the tree
disp(['---Tree---']);
disp(['        Q1        Q2        Q3        Q4  Parent Node']);
disp(['   -------   -------   -------   -------    ------']);
disp(qTree);
disp(['Number Of Nodes in Tree: ', num2str(size(qTree,1))]);

%Calculate shortest path in the tree
qMilestones=getPath(qTree);
disp(['qMileStones:']);
disp(['        Q1        Q2        Q3        Q4']);
disp(['   -------   -------   -------   -------']);
disp(qMilestones);
disp(['Number Of qMileStones: ', num2str(size(qMilestones,1))]);

end


function qMilestones=getPath(qTree)

qMilestones=[];
%Set stating point as last node (target)
node=size(qTree,1);

%Iterate till it reaches source node
while node>0
    
    currQ=qTree(node,1:4);
    node=qTree(node,5);
    
    qMilestones=[qMilestones;currQ];
end

%Reverse the list to show from qStart to target
qMilestones=flipud(qMilestones);

end


function qCurrOnTree=findNearest (qRand,qTree)

smallestDist=inf;
noOfNodes=size(qTree,1);

for elemCnt=1:1:noOfNodes
    currDistance=norm(qRand-qTree(elemCnt,1:4));
    
    if currDistance<smallestDist
        smallestElementIndex=elemCnt;  
        smallestDist=currDistance;
    end
end

qCurrOnTree=[qTree(smallestElementIndex,1:4) smallestElementIndex];

end


function [colStruct]=checkCollision(rob,rob3Link,qCurr,qNear,sphereCenter,sphereRadius)

%Extract joint angles
qNearEF=qNear;
qNearL1=qNear(1:3);
qCurrEF=qCurr;
qCurrL1=qCurr(1:3);

%Extract Positions of the end of the two arms
posNearEF=rob.fkine(qNearEF);posNearEF=posNearEF(1:3,4);
posNearL1=rob3Link.fkine(qNearL1);posNearL1=posNearL1(1:3,4);
posCurrEF=rob.fkine(qCurrEF);posCurrEF=posCurrEF(1:3,4);
%posCurrL1=rob3Link.fkine(qCurrL1);posCurrL1=posCurrL1(1:3,4);

%Check if the new configuration denotes a point outside the work-space [-1 1]
for indx=1:1:3
    if abs(posNearEF(indx)) > 1 || abs(posNearL1(indx)) > 1
        colStruct.colsn=1;
        colStruct.EFPos=posCurrEF;
        return;
    end
end

%Check if the arms or EF of the new configuration collide with the obstacle 
pointCollides=checkCollisionAtPoints(posNearEF,posNearL1,posCurrEF,sphereCenter,sphereRadius);

    if pointCollides==1
        colStruct.colsn=1;
    else
        colStruct.colsn=0;
    end

colStruct.EFPos=posNearEF;

end


function pointCollides=checkCollisionAtPoints(posNearEF,posNearL1,posCurrEF,sphereCenter,sphereRadius)

pointCollides=0;

%Check for the end of the arms colliding with obstacle
if ((norm(posNearL1 - sphereCenter) < sphereRadius) || ...
    (norm(posNearEF - sphereCenter) < sphereRadius) || ...
    (norm((posNearL1+posNearEF) - sphereCenter)< sphereRadius))
    
    pointCollides=1;
else
    %Check for 20 equally spaced points along the arms to check for collision
    for step=0:0.05:1
        %Points along the robot arms itself from joint to joint
        posNearPointsBaseToL1= moveStep([0; 0; 0],posNearL1,step);
        posNearPointsL1ToEF= moveStep(posNearL1,posNearEF,step);
        posNearPointsCurrToNearEF= moveStep(posCurrEF,posNearEF,step);
        posNearPointsBaseToEF= moveStep([0; 0; 0],(posNearL1+posNearEF),step);

        %If any of the points on the path or the arm itself are inside the
        %sphere (at <= radius distance to the centre of sphere
        if ((norm(posNearPointsBaseToL1 - sphereCenter)< sphereRadius) || ...
            (norm(posNearPointsL1ToEF - sphereCenter)< sphereRadius) || ...
            (norm(posNearPointsCurrToNearEF - sphereCenter)< sphereRadius) || ...
            (norm(posNearPointsBaseToEF - sphereCenter) < sphereRadius))

            pointCollides=1;
            break;
        else
            continue;
        end
    end
end
end
   

function posNew=moveStep(posA, posB, step)
    %Calculate increment size based on the distance of the points
    increment=(norm(posB-posA) * step);
    %Move a step towards PosB from PosA
    posNew = posA + ( ((posB-posA)/norm(posB-posA)) * increment );
end


function q = ikine4Joint(f,qInit,posGoal)
%Initialize the current angles and positions
qCurr = qInit;
posInit= f.fkine(qInit);
posInit=posInit(1:3,4);         %Extract the co-ordinates of the starting position of end-effector
posError= posGoal-posInit;      %Compute the initial error between the start and end postions of the end-effector
errLimit=0.001;                 %Define a small arbitrary error limit
stepSize=0.05;                  %Define a step size

while norm(posError)>errLimit
    posCurr=f.fkine(qCurr);     %Extract ith position in ith iteration
    posCurr=posCurr(1:3,4);     %Extract the initial co-ordinates
    posError = posGoal-posCurr; %Extract position error in the ith iteration
    jacob = jacob0(f,qCurr);    %Calculate Jacobian
    jacob = jacob(1:3,1:4);     %Exlude the angular velocity components
    qErr = qInit-qCurr;         %Calculate the qErr
    qDelta = pinv(jacob)*posError ...
                + (eye(4) - (pinv(jacob)*jacob))*qErr';  %Include the Null Space Projection matrix (I - J#J)
    qCurr = qCurr + (stepSize*qDelta)';                  %Calculate the target join angles in the ith iteration
end
   q=qCurr;
end

