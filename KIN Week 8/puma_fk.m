function [pos, R] = puma_fk(theta1, theta2, theta3, theta4, theta5, theta6)
%PUMA_FK The input to the function will be the joint angles of the robot in radians.
%    The output must contain end effector position of the robot arm and the rotation matrix representing the rotation from frame
%    6 to frame 0, as specified in the question.

    %% Your code goes here
    % Robot geometric params
    a = 13;
    b = 2.5;
    c = 8;
    d = 2.5;
    e = 8;
    f = 2.5;
    
    % DH table
    n = 6; 
    theta = [theta1, theta2, theta3, theta4, theta5, theta6];
    dh_table = [
        pi/2,       0,      a,      theta(1)
        0,          c,      -b,     theta(2)
        -pi/2,      0,      -d,     theta(3)
        pi/2,       0,      e,      theta(4)
        -pi/2,      0,      0,      theta(5)
        0,          0,      f,      theta(6)];
    
    % Forward kinematics
    Ti = cell(n, 1);
    for i = 1:n
        if i == 1
            Ti{i} = homoTrans(dh_table(i, :));
        else
            Ti{i} = Ti{i - 1} * homoTrans(dh_table(i, :));
        end
    end
    
    R = Ti{n}(1: 3, 1: 3);
    pos = Ti{n}(1: 3, end)';
    
end

%% Helper function
function T = homoTrans(dh_row)
    a = dh_row(2);
    alpha = dh_row(1);
    d = dh_row(3);
    theta = dh_row(4);
        
    T = [
        cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)
        sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)
        0, sin(alpha), cos(alpha), d
        0, 0, 0, 1]; 
end