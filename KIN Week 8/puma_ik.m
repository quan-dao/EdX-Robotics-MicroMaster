function [ ik_sol ] = puma_ik( x, y, z, R )
%PUMA_IK Write your code here. The input to the function will be the position of
%    the end effector (in inches) in the world frame, and the 
%    Rotation matrix R_60 as described in the question.
%    The output must be the joint angles of the robot to achieve 
%    the desired end effector position and orientation.

    %% YOUR CODE GOES HERE
    
    %% Wrist positioning
    % Wrist position in global frame
    p0C = [x; y; z] - 2.5 * R(1: 3, end)
    xc = p0C(1);
    yc = p0C(2);
    zc = p0C(3);
    
    % find theta3
    if (xc^2 + yc^2 - 25) < 0
        delta = 0;
    else
        delta = sqrt(xc^2 + yc^2 - 25) / 8;
    end
    theta3 = asin(1 - 0.5 * delta^2 - 0.5 * ((zc - 13) / 8)^2);
    
    % find theta2
    c3 = cos(theta3);
    s3 = sin(theta3);
    if s3 == 1
        theta2 = 0;  % indeed, any value will do
    else
        s2 = (delta * c3 + (zc - 13) * (s3 - 1) / 8) / (2 * (s3 - 1));
        c2 = (delta * (s3 - 1) - (zc - 13) * c3 / 8) / (2 * (s3 - 1));
        theta2 = atan2(s2, c2);
    end
    
    % find theta1
    theta1 = atan2(yc, xc) - atan2(5/8, delta);
    
    %% End-effector orienting
    R03 = oriFrame3(theta1, theta2, theta3);
    R36 = R03' * R;
    
    theta5 = acos(R36(3, 3));
    
    if theta5 >= 0
        theta4 = atan2(-R36(2, 3), -R36(1, 3));
        theta6 = atan2(-R36(3, 2), R36(3, 1));
    else
        theta4 = atan2(R36(2, 3), R36(1, 3));
        theta6 = atan2(R36(3, 2), -R36(3, 1));
    end
    
    ik_sol = [theta1, theta2, theta3, theta4, theta5, theta6];

end

%% helper function
function R03 = oriFrame3(theta1, theta2, theta3)
    R03 = [
        cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3),     -sin(theta1),   - cos(theta1)*cos(theta2)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)
        cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3),      cos(theta1),    - cos(theta2)*sin(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2)
        cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2),                              0,              cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)];
end