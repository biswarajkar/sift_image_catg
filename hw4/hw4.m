% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% input: questionNum -> Integer between 1 and 5 that denotes question
%                       number to run.
function hw4(questionNum)

%     close all;

    % define time horizon and time step
    T = 100;
    dt = 0.1;
    
    % define system parameters
    b = 0.1;
    m = 1;
    A = [1 0 dt 0; 0 1 0 dt; 0 0 (1 - dt*b/m) 0; 0 0 0 (1 - dt*b/m)];
    B = [0 0; 0 0; dt 0; 0 dt];

    % define cost function
    QT = 1000*eye(4);
    Q = 0.1*eye(4);
    R = eye(2);

    % initial state (4x1)
    x0 = [0;1;1;0];
    
    % ************* Question 1 *******************
    if questionNum == 1
        [xFH,uFH] = Q1(A,B,QT,Q,R,T,x0);
        figure;
        plot(xFH(1,:),xFH(2,:),'gx');
        figure;
        plot(uFH');
    end
    
    
    % ************* Question 2 *******************
    if questionNum == 2
        [xFH,uFH] = Q1(A,B,QT,Q,R,T,x0);
        [xRH,uRH] = Q2(A,B,QT,Q,R,T,x0);
        figure;
        plot(xFH(1,:),xFH(2,:),'gx');
        hold on;
        plot(xRH(1,:),xRH(2,:),'rx');
    end
    
    
    % ************* Question 3 *******************
    if questionNum == 3
        Rdelta = 10*eye(2); % cost of a change in control input
        [xIC,uIC] = Q3(A,B,QT,Q,R,Rdelta,T,x0);
        figure;
        plot(xIC(1,:),xIC(2,:),'gx');
        
        figure;
        plot(xIC(5:6,:)');
    end
    

    
end



