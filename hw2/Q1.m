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
qCurr = qStart;
qNewNode=qStart;

posInit= rob.fkine(qStart);
posInit=posInit(1:3,4);        %Extract the co-ordinates of the starting position of end-effector
posErrFromGoal= (posGoal-posInit);   %Compute the initial error between the start and end postions of the end-effector
errLimit=0.01;                 %Define a small arbitrary error limit
stepSize=0.5;                  %Define a step size
runLimit=10;
qRanErrDist=1;
qTree=[qStart,0];

for counter=1:1:runLimit
    
        posErrRandfromGoal=posErrFromGoal+qRanErrDist;
        while norm(posErrFromGoal)<norm(posErrRandfromGoal)

            %Generate a random configuration
            qRand=(-pi + (2*pi)*rand(1,4));
            
            %Find the closest configuration in the existing tree to the
            %random configuration
            qCurrOnTree= findNearest(qRand,qTree);
            qCurr=qCurrOnTree(1,1:4);
            indexOnTree=qCurrOnTree(1,5);
            
            %Generate a configuration at 'stepSize' distance from the tree 
            %towards the random configuration
            qNear=qCurr+(stepSize*((qRand-qCurr)/norm(qRand-qCurr)));
            
            %Check if the new configuration collides with the obstacle
            [collision] = checkCollision(rob,qCurr,qNear,sphereCenter,sphereRadius,posGoal);
            posErrRandfromGoal=collision.EFPos-posGoal;

            %If there is no collision of the new configuration, add it to
            %the tree
            if (collision.colsn==0)
                posErrFromGoal=posErrRandfromGoal;
                parentIndex=indexOnTree;
                qNewNode=qNear;
                break;
            end
        end
        qTree(counter,1:4)=qNewNode;
        qTree(counter,5)=parentIndex;
        
        disp(qTree);
        %Calculate Path (TODO)
        qMilestones(counter,:)=qNewNode;
        
        %Check if EF of the last milestone is close enough to the goal
        if norm(posErrFromGoal)<errLimit
            break
        end
end

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
    
    qCurrOnTree=qTree(smallestElementIndex,:);
end

end

function [colStruct]=checkCollision(rob,qCurr,qNear,sphereCenter,sphereRadius,posGoal)

%Extract joint angles
qRandEF=qNear;
qRandL1=[qNear(1:3) 0];

qCurrEF=qCurr;
qCurrL1=[qCurr(1:3) 0];


%Extract Positions of the end of the two arms
posRandEF=rob.fkine(qRandEF);posRandEF=posRandEF(1:3,4);
posRandL1=rob.fkine(qRandL1);posRandL1=posRandL1(1:3,4);

posCurrEF=rob.fkine(qCurrEF);posCurrEF=posCurrEF(1:3,4);
posCurrL1=rob.fkine(qCurrL1);posCurrL1=posCurrL1(1:3,4);

%Check if the new configuration denotes a point outside the work-space [-1 1]
for indx=1:1:3
    if abs(posRandEF(indx)) > 1 || abs(posRandL1(indx)) > 1
        colStruct.colsn=1;
        colStruct.EFPos=posCurrEF;
        return;
    end
end

%Check if the arms or EF of the new configuration collide with the obstacle 
pointCollides=checkCollisionAtPoints(posRandEF,posRandL1,sphereCenter,sphereRadius);

    if pointCollides==1
        colStruct.colsn=1;
    else
        colStruct.colsn=0;
    end

colStruct.EFPos=posRandEF;

end

function pointCollides=checkCollisionAtPoints(posRandEF,posRandL1,sphereCenter,sphereRadius)

%Check for 20 equally spaced points along the arms to check for collision
for step=0:0.05:1
    %Points along the robot arms itself from joint to joint
    posPointRandBaseToL1= moveStep([0; 0; 0],posRandL1,step);
    posPointRandL1toEF= moveStep(posRandL1,posRandEF,step);
    
    %If any of the points on the path or the arm itself are inside the
    %sphere (at <= radius distance to the centre of sphere
    if ((norm(posRandEF - sphereCenter)<= sphereRadius) || ...
        (norm(posPointRandBaseToL1 - sphereCenter)<= sphereRadius) || ...
        (norm(posPointRandL1toEF - sphereCenter)<= sphereRadius))

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

