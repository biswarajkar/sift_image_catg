% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First six joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)
function q = Q5(f1,f2,qInit,f1Target,f2Target)

%Initialize the current angles and positions
qCurr = qInit;
posInitArm1 = f1.fkine([qInit(1:7) qInit(8:9)]);
posInitArm2 = f2.fkine([qInit(1:7) qInit(10:11)]);
posInitArm1 = posInitArm1(1:3,4);         %Extract the co-ordinates of the starting position of end-effector for Arm 1
posInitArm2 = posInitArm2(1:3,4);         %Extract the co-ordinates of the starting position of end-effector for Arm 2

posGoal = [f1Target;f2Target];            %Define the composite target/goal position vector

posError = posGoal - [posInitArm1 ; posInitArm2];     %Define position error as the difference of final and initial positions
errLimit = 0.001;                         %Define a small arbitrary error limit
stepSize = 0.05;                          %Define a step size

while norm(posError)>errLimit
    posCurrArm1 = f1.fkine([qCurr(1:7) qCurr(8:9)]);     % Joints 1-7 are common, 8-9 are for finger/arm 1
    posCurrArm2 = f2.fkine([qCurr(1:7) qCurr(10:11)]);   % Joints 1-7 are common, 10-11 are for finger/arm 2
    
    posError = posGoal - [posCurrArm1(1:3,4) ; posCurrArm2(1:3,4)];
    
    jacobArm1 = jacob0(f1,[qCurr(1:7) qCurr(8:9)]);
    jacobArm2 = jacob0(f2,[qCurr(1:7) qCurr(10:11)]);
    
    jacobArm1 = [jacobArm1(1:3,1:9) [0 0;0 0;0 0]];                     %Exlude the angular velocity components for Arm 1
    jacobArm2 = [jacobArm2(1:3,1:7) [0 0;0 0;0 0] jacobArm2(1:3,8:9)];  %Exlude the angular velocity components for Arm 2
    
    Jcombined = [jacobArm1 ;jacobArm2];                  %Combine the jacobian of the 2 arms/fingers to create a composite jacobian
    
    qErr = qInit - qCurr;                                %Calculate the qErr
    
    qDelta = pinv(Jcombined)*posError
                         + (eye(11) - (pinv(Jcombined)*Jcombined))*qErr';  %Include the Null Space Projection matrix (I - J#J)
    qCurr = qCurr + (stepSize*qDelta)';                  %Calculate the target join angles in the ith iteration
end
    q=qCurr;

end


    
