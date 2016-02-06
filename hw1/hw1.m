% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% input: questionNum -> Integer between 1 and 5 that denotes question
%                       number to run.
function hw1(questionNum)

    if nargin < 1
        error('Error: please enter a question number as a parameter');
    end
    
    % set up robot and initial joint configuration
	[f1, f2] = createRobot();
	qn = [0 -0.78 0 1.57 0 1.57 0];
	q1Init = [qn -1 1];
	q2Init = [qn 1 -1];
	spherePos = [0.65;0;-0.75];

    % ************* Question 1 *******************
    if questionNum == 1
        close all;
        figure;
        f1.plot(q1Init,'jointdiam',1);
        hold on;	
        drawSphere(spherePos,0.1);

        % TODO: you implement this function (Q1)
        q1Ball = Q1(f1,spherePos);
%         q1Ball = Q1_answer(f1,spherePos);

        t = [0:0.1:2]';
        traj = jtraj(q1Init, q1Ball, t);
        f1.plot(traj);
    end

    % ************* Question 2 *******************
    if questionNum == 2
        close all;
        figure;
        f1.plot(q1Init,'jointdiam',1);
        hold on;	
        drawSphere(spherePos,0.1);

        % TODO: you implement this function (Q2)
        q1Ball = Q2(f1,q1Init,spherePos);
%        q1Ball = Q2_answer(f1,q1Init,spherePos);

        t = [0:0.05:2]';
        traj = jtraj(q1Init, q1Ball, t);
        f1.plot(traj);
    end	
    
    % ************* Question 3 *******************
    if questionNum == 3
        close all;
        figure;
        f1.plot(q1Init,'jointdiam',1);
        hold on;	
        drawSphere(spherePos,0.1);

        % TODO: you implement this function (Q2)
        q1Ball = Q3(f1,q1Init,spherePos);
%        q1Ball = Q3_answer(f1,q1Init,spherePos);

        t = [0:0.05:2]';
        traj = jtraj(q1Init, q1Ball, t);
        f1.plot(traj);
    end	

    % ************* Question 4 *******************
    if questionNum == 4
        close all;
        figure;
        f1.plot(q1Init,'jointdiam',1);
        hold on;	
        f2.plot(q2Init,'jointdiam',1);
        drawSphere(spherePos,0.1);    

        % TODO: you implement this function (Q4)
        qInit = [q1Init(1:9) q2Init(8:9)];
        qTarget =  Q4(f1,f2,qInit,spherePos+[0;-0.1;0.0],spherePos+[0;0.1;-0.0]);
%        qTarget =  Q4_answer(f1,f2,qInit,spherePos+[0;-0.1;0.0],spherePos+[0;0.1;-0.0]);

        t = [0:0.05:2]';
        traj = jtraj(qInit, qTarget, t);
        for q=traj'
            f1.plot(q(1:9)','jointdiam',1);
            f2.plot([q(1:7)' q(10:11)'],'jointdiam',1);
        end
    end

    % ************* Question 5 *******************
    if questionNum == 5
        close all;
        figure;
        f1.plot(q1Init,'jointdiam',1);
        hold on;	
        f2.plot(q2Init,'jointdiam',1);
        drawSphere(spherePos,0.1);    

        % TODO: you implement this function (Q4)
        qInit = [q1Init(1:9) q2Init(8:9)];
%        qTarget =  Q5(f1,f2,qInit,spherePos+[0;-0.1;0.0],spherePos+[0;0.1;-0.0]);
        qTarget =  Q5(f1,f2,qInit,spherePos+[0;-0.1;0.0],spherePos+[0;0.1;-0.0]);

        t = [0:0.05:2]';
        traj = jtraj(qInit, qTarget, t);
        for q=traj'
            f1.plot(q(1:9)','jointdiam',1);
            f2.plot([q(1:7)' q(10:11)'],'jointdiam',1);
        end
    end
    
        
end

    
	function [f1, f2] = createRobot()
	
		L(1) = Link([0 0 0 1.571]);
		L(2) = Link([0 0 0 -1.571]);
		L(3) = Link([0 0.4318 0 -1.571]);
		L(4) = Link([0 0 0 1.571]);
		L(5) = Link([0 0.4318 0 1.571]);
		L(6) = Link([0 0 0 -1.571]);
		L(7) = Link([0 0 0 0]);
		L(8) = Link([0 0 0.2 0]);
		L(9) = Link([0 0 0.2 0]);

		f1 = SerialLink(L, 'name', 'f1');
		f2 = SerialLink(L, 'name', 'f2');
	
	end
	
	function drawSphere(position,diameter)

		diameter = 0.1;
		[X,Y,Z] = sphere;
		X=X*diameter;
		Y=Y*diameter;
		Z=Z*diameter;
		X=X+position(1);
		Y=Y+position(2);
		Z=Z+position(3);
		surf(X,Y,Z);
		%~ shading flat

	end

