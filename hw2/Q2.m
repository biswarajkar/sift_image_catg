% BISWARAJ KAR - CS 5335
% Smooth path given in qMilestones
% input: qMilestones -> nx4 vector of n milestones. 
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xm vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You should output a number of
%                    milestones m<=n.
function qMilestonesSmoothed = Q2(rob,qMilestones,sphereCenter,sphereRadius)
disp('/----Smoothing----\');
%Create a 3-link robot for computing position of end of L1
L1(1) = Link([0 0 0 1.571]);
L1(2) = Link([0 0 0 -1.571]);
L1(3) = Link([0 0.4318 0 -1.571]);
rob3Link = SerialLink(L1,'name','robot');

qMilestonesSmoothed = smoothenPath(rob,rob3Link,qMilestones,sphereCenter,sphereRadius);

disp(['        Q1        Q2        Q3        Q4']);
disp(['   -------   -------   -------   -------']);
disp(qMilestonesSmoothed);
disp(['Number Of qMileStones after smoothing: ', num2str(size(qMilestonesSmoothed,1))]);

end
    
function qMilestonesSmoothed = smoothenPath(rob,rob3Link,qMilestones,sphereCenter,sphereRadius)

noOfMilestones = size(qMilestones,1);
qSmMilestone = qMilestones(1,:);

for milestoneNum = 1:noOfMilestones
    for nextMilestoneNum = milestoneNum+1:20
        if(nextMilestoneNum>noOfMilestones)
            break;
        end
        
        %Get current and next configurations
        qCurr=qMilestones(milestoneNum,:);
        qNext=qMilestones(nextMilestoneNum,:);
        
        %Check if the path between the current and next configuration is
        %clear
        collision = checkCollision(rob,rob3Link,qCurr,qNext,sphereCenter,sphereRadius);

        if collision==0
            if (milestoneNum+nextMilestoneNum)==noOfMilestones
                qSmMilestone = [qSmMilestone ; qMilestones(noOfMilestones,:)];
                milestoneNum=noOfMilestones;
                break;
            end
        else
            %Set previous q as successful q and reject the q which collided
            qSmMilestone =[qSmMilestone ; qMilestones(nextMilestoneNum-1,:)];
            milestoneNum=nextMilestoneNum-1;
            break;
        end
    end
    qMilestonesSmoothed=qSmMilestone;
end
end
    
function collision=checkCollision(rob,rob3Link,qCurr,qNext,sphereCenter,sphereRadius)

collision=0;

%Extract Joint Angles of the two arms for current and next q
qCurrL1=qCurr(1:3);
qCurrEF=qCurr;
qNextL1=qNext(1:3);
qNextEF=qNext;

%Extract Positions of the end of the two arms for current and next q
posCurrL1=rob3Link.fkine(qCurrL1);posCurrL1=posCurrL1(1:3,4);
posCurrEF=rob.fkine(qCurrEF);posCurrEF=posCurrEF(1:3,4);
posNextL1=rob3Link.fkine(qNextL1);posNextL1=posNextL1(1:3,4);
posNextEF=rob.fkine(qNextEF);posNextEF=posNextEF(1:3,4);

%Check 20 points along the path from current to next position for collision
for step=0:0.05:1
    
    posNearPointsL1C2N= moveStep(posCurrL1,posNextL1,step);
    posNearPointsEFC2N= moveStep(posCurrEF,posNextEF,step);
    posNearPointsBase2EFC2N= moveStep((posCurrL1+posCurrEF),(posNextL1+posNextEF),step);
    
    %If any of the points on the path from current to next position are
    %inside the sphere (at <= radius distance to the centre of sphere)
    if ((norm(posNearPointsL1C2N - sphereCenter)< sphereRadius) || ...
        (norm(posNearPointsEFC2N - sphereCenter)< sphereRadius) || ...
        (norm(posNearPointsBase2EFC2N - sphereCenter) < sphereRadius))
        
        collision=1;
        break;
    else
        continue;
    end
end
    
end

function posNew=moveStep(posA, posB, step)
    %Calculate increment size based on the distance of the points
    increment=(norm(posB-posA) * step);
    %Move a step towards PosB from PosA
    posNew = posA + ( ((posB-posA)/norm(posB-posA)) * increment );
end

