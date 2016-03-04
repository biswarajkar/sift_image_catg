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
runLimit=50;
qRanErrDist=1;
sphereRadius=sphereRadius+1;


%posErrRandfromGoal=posErrFromGoal+qRanErrDist;
for counter=1:1:runLimit

        posErrRandfromGoal=posErrFromGoal+qRanErrDist;
        disp('Prev Node: ');disp(qCurr); %Debug Only, remove
        while norm(posErrFromGoal)<norm(posErrRandfromGoal)

            %Generate a random arm configuration
            qRand=qCurr+(-stepSize + (2*stepSize)*rand(1,4));

            %Check if the new random configuration collides with the obstacle
            [collision] = checkCollision(rob,qCurr,qRand,sphereCenter,sphereRadius,posGoal);
            disp('Collision? ');disp(collision.colsn);   
            posErrRandfromGoal=collision.EFPos-posGoal;

            %If there is no collision of the new configuration and the new
            %position is closer to the goal than the existing position
            if (collision.colsn==0) && (norm(posErrRandfromGoal) < norm(posErrFromGoal))
                disp('qRand:');disp(qRand);                         %Debug Only, remove
                disp('Dist Init:');disp(norm(posErrFromGoal));      %Debug Only, remove
                disp('Dist Rand:');disp(norm(posErrRandfromGoal));  %Debug Only, remove
                qCurr=qRand;
                posErrFromGoal=posErrRandfromGoal;
                qNewNode=qRand;
                break;
            end
        end
        qMilestones(counter,:)=qNewNode;
        
        disp('Milestone: ');disp(counter); %Debug Only, remove
        disp('-------------------------------');%Debug Only, remove
        
        if norm(posErrFromGoal)<errLimit
            break
        end
end

end
    
function [colStruct]=checkCollision(rob,qCurr,qRand,sphereCenter,sphereRadius,posGoal)

%Extract joint angles
qRandEF=qRand;
qRandL2=[qRand(1:3) 0];
qRandL1=[qRand(1:2) 0 0];

qCurrEF=qCurr;
qCurrL2=[qCurr(1:3) 0];
qCurrL1=[qCurr(1:2) 0 0];

%Extract Positions
posRandEF=rob.fkine(qRandEF);posRandEF=posRandEF(1:3,4);
posRandL2=rob.fkine(qRandL2);posRandL2=posRandL2(1:3,4);
posRandL1=rob.fkine(qRandL1);posRandL1=posRandL1(1:3,4);

posCurrEF=rob.fkine(qCurrEF);posCurrEF=posCurrEF(1:3,4);
posCurrL2=rob.fkine(qCurrL2);posCurrL2=posCurrL2(1:3,4);
posCurrL1=rob.fkine(qCurrL1);posCurrL1=posCurrL1(1:3,4);


    %Check every point 0.1 units apart along the robot arm or the path
    for pointIncrement=0:0.1:1
        pointCollides=checkCollisionAtPoint(posRandEF,posRandL2,posRandL1,...
                              posCurrEF,posCurrL2,posCurrL1,...
                              sphereCenter,sphereRadius,pointIncrement);
        if pointCollides==1
            colStruct.colsn=1;
            colStruct.EFPos=posRandEF;
            break; 
        else
            colStruct.colsn=0;
        end
        
    end
    
    colStruct.EFPos=posRandEF;
    %colStruct.colsn=0;

end

function pointCollides= checkCollisionAtPoint(posRandEF,posRandL2,posRandL1,...
                          posCurrEF,posCurrL2,posCurrL1,...
                          sphereCenter,sphereRadius,step)
%Points along the robot arms itself from joint to joint
posPointRandL1toL2= moveStep(posRandL1,posRandL2,step);
posPointRandL2toEF= moveStep(posRandL2,posRandEF,step);

%Arm end position points from current to random point along path
posPointL2C2R= moveStep(posCurrL2,posRandL2,step);
posPointEFC2R= moveStep(posCurrEF,posRandEF,step);

%If any of the points on the path or the arm itself are inside the
%sphere (at <= radius distance to the centre of sphere
if ((norm(posRandEF - sphereCenter)<= sphereRadius) || ...
   (norm(posRandL2 - sphereCenter)<= sphereRadius) || ...
   (norm(posRandL1 - sphereCenter)<= sphereRadius) || ...
   (norm(posPointRandL1toL2 - sphereCenter)<= sphereRadius) || ...
   (norm(posPointRandL2toEF - sphereCenter)<= sphereRadius) || ...
   (norm(posPointL2C2R - sphereCenter)<= sphereRadius) || ...
   (norm(posPointEFC2R - sphereCenter)<= sphereRadius))

    pointCollides=1;
else
    pointCollides=0;
end              
                      
end
                      

function posNew=moveStep(posA, posB, increment)
    for coor=1:1:3
        if posA(coor) <= posB(coor)
            posNew(coor) = posA(coor) + ((posB(coor)-posA(coor))*increment);
        else
            posNew(coor) = posB(coor) + ((posA(coor)-posB(coor))*increment);
        end
    end
    posNew=posNew';
end

