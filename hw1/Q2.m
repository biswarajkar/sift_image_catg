% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q2(f,qInit,posGoal)
%Initialize the current angles and positions
qCurr = qInit;
posInit = f.fkine(qInit);
posInit = posInit(1:3,4);         %Extract the co-ordinates of the starting position of end-effector
posError = posGoal - posInit;     %Compute the initial error between the start and end postions of the end-effector
errLimit = 0.001;                 %Define a small arbitrary error limit
stepSize = 0.05;                  %Define a step size

while norm(posError)>errLimit
    posCurr = f.fkine(qCurr);     %Extract ith position in ith iteration
    posCurr = posCurr(1:3,4);     %Extract the initial co-ordinates
    posError = posGoal-posCurr;   %Extract position error in the ith iteration
    jacob = jacob0(f,qCurr);
    jacob = jacob(1:3,1:9);       %Exlude the angular velocity components
    qDelta = pinv(jacob)*posError;
    qCurr = qCurr + (stepSize*qDelta)'; %Calculate the angles in the ith iteration
end
   q=qCurr;
end


