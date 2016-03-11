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
%Initialize the current angles and positions
posGoal=xGoal;

posInit= rob.fkine(qStart);
posInit=posInit(1:3,4);        %Extract the co-ordinates of the starting position of end-effector
posErrFromGoal= (posGoal-posInit);   %Compute the initial error between the start and end postions of the end-effector
errLimit=0.04;                 %Define a small arbitrary error limit
stepSize=0.5;                  %Define a step size
runLimit=200;
qRanErrDist=1;
qTree=[qStart,0];
posErrqNearGoal=inf;

for counter=1:1:runLimit
    
%while norm(posErrqNearGoal)>errLimit
        
        %Generate a random configuration
        if size(qTree,1)>20 %rem(size(qTree,1),50)==0
            qRand=ikine4Joint(rob,qStart,posGoal);
        else
            qRand=(-pi + (2*pi)*rand(1,4));
        end
        
        %Find the closest configuration in the existing tree to the
        %random configuration
        qCurrOnTree= findNearest(qRand,qTree);
        qCurr=qCurrOnTree(1,1:4);
        
        %Generate a configuration at 'stepSize' distance from the tree
        %towards the random configuration
        qNear=qCurr+(stepSize*((qRand-qCurr)/norm(qRand-qCurr)));
        
        %Check if the new configuration collides with the obstacle
        [collision] = checkCollision(rob,qCurr,qNear,sphereCenter,sphereRadius);
        
        %If there is no collision of the new configuration, add it to
        %the tree
        if (collision.colsn==0)
            posErrqNearGoal=collision.EFPos-posGoal;
            qNearWithParent=[qNear qCurrOnTree(1,5)];
            qTree=[qTree;qNearWithParent];
            %break;
        end
    %end
    
    
    
end

disp(['Tree:']);disp(size(qTree,1));
disp(qTree);

%Calculate Path in the tree
qMilestones=getPath(qTree);
disp(['qMileStone:']);disp(qMilestones);

end

function qMilestones=getPath(qTree)

%Set stating point as 
qMilestones=[];
node=size(qTree,1);

while node>0
    
    currQ=qTree(node,1:4);
    node=qTree(node,5);
    
    qMilestones=[qMilestones;currQ];
end

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


function [colStruct]=checkCollision(rob,qCurr,qNear,sphereCenter,sphereRadius)

%Extract joint angles
qNearEF=qNear;
qNearL1=[qNear(1:3) 0];

qCurrEF=qCurr;
qCurrL1=[qCurr(1:3) 0];

%Extract Positions of the end of the two arms
posNearEF=rob.fkine(qNearEF);posNearEF=posNearEF(1:3,4);
posNearL1=rob.fkine(qNearL1);posNearL1=posNearL1(1:3,4);

posCurrEF=rob.fkine(qCurrEF);posCurrEF=posCurrEF(1:3,4);
posCurrL1=rob.fkine(qCurrL1);posCurrL1=posCurrL1(1:3,4);

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

%Check for 20 equally spaced points along the arms to check for collision
for step=0:0.05:1
    %Points along the robot arms itself from joint to joint
    posNearPointsBaseToL1= moveStep([0; 0; 0],posNearL1,step);
    posNearPointsL1ToEF= moveStep(posNearL1,posNearEF,step);
    posNearPointsCurrToNearEF= moveStep(posCurrEF,posNearEF,step);
    posNearPointsBaseToNearEF= moveStep([0; 0; 0],(posNearL1+posNearEF),step);
    
    %If any of the points on the path or the arm itself are inside the
    %sphere (at <= radius distance to the centre of sphere
    if ((norm(posNearEF - sphereCenter)<= sphereRadius) || ...
        (norm((posNearL1+posNearEF) - sphereCenter)<= sphereRadius) || ...
        (norm(posNearPointsBaseToL1 - sphereCenter)<= sphereRadius) || ...
        (norm(posNearPointsL1ToEF - sphereCenter)<= sphereRadius) || ...
        (norm(posNearPointsCurrToNearEF - sphereCenter)<= sphereRadius))

        pointCollides=1;
        break;
    else
        pointCollides=0;
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
qInst = qInit;
error = [1 1 1];
while(norm(error) > 0.001)
    instPos = f.fkine(qInst);
    error = posGoal - instPos(1:3,4);
    J = jacob0(f,qInst);
    qChange = pinv(J(1:3,:)) *  error;
    qInst = qInst + 0.05 * qChange';
end
q = qInst;
end

