function [ ik_sol ] = puma_ik( x, y, z, R )
%PUMA_IK Write your code here. The input to the function will be the position of
%    the end effector (in inches) in the world frame, and the 
%    Rotation matrix R_60 as described in part 2.
%    The output must be the joint angles of the robot to achieve 
%    the desired end effector position and orientation.

    
    %% Wrist position in global frame
    p0C = [x; y; z] - 2.5 * R(1: 3, end);
    xc = p0C(1);
    yc = p0C(2);
    zc = p0C(3);
    
    % find theta3
    if (xc^2 + yc^2 - 25) < 0
        delta = 0;
    else
        delta = sqrt(xc^2 + yc^2 - 25) / 8;
    end
    theta3_1 = asin(1 - 0.5 * delta^2 - 0.5 * ((zc - 13) / 8)^2);
    theta3_2 = pi - theta3_1;
    arr_theta3 = [theta3_1; theta3_2];  % 1st brand
    
    % find theta2
    arr_theta2 = zeros(2, 2);
    for i = 1 : 2
        theta3 = arr_theta3(i);
        c3 = cos(theta3);
        s3 = sin(theta3);
        if s3 == 1
            theta2_i1 = 0;  % indeed, any value will do
            theta2_i2 = 0;
        else
            % delta > 0
            s21 = (delta * c3 + (zc - 13) * (s3 - 1) / 8) / (2 * (s3 - 1));
            c21 = (delta * (s3 - 1) - (zc - 13) * c3 / 8) / (2 * (s3 - 1));
            theta2_i1 = atan2(s21, c21);
            % delta < 0
            s22 = (-delta * c3 + (zc - 13) * (s3 - 1) / 8) / (2 * (s3 - 1));
            c22 = (-delta * (s3 - 1) - (zc - 13) * c3 / 8) / (2 * (s3 - 1));
            theta2_i2 = atan2(s22, c22);
        end
        arr_theta2(i, :) = [theta2_i1, theta2_i2];
    end
    
    % find theta1
    arr_theta1 = zeros(2, 1);
    arr_theta1(1) = atan2(yc, xc) - atan2(5/8, delta);
    arr_theta1(2) = atan2(yc, xc) - atan2(5/8, -delta);
    
    %% End-effector orienting
    arr_theta5 = zeros(4, 2);
    for i = 1 : 2
        theta1 = arr_theta1(i);
        theta3 = arr_theta3(i);
        for j = 1 : 2
            theta2 = arr_theta2(i, j);
            R03 = oriFrame3(theta1, theta2, theta3);
            R36 = R03' * R;
            arr_theta5(j + (i - 1) * 2, :) = [acos(R36(3, 3)), -acos(R36(3, 3))];
        end
    end
    
    arr_theta4 = zeros(4, 2);
    arr_theta6 = zeros(4, 2);
    for i = 1 : 4
        for j = 1: 2
            theta5 = arr_theta5(i, j);
            if theta5 >= 0
                theta4 = atan2(-R36(2, 3), -R36(1, 3));
                theta6 = atan2(-R36(3, 2), R36(3, 1));
            else
                theta4 = atan2(R36(2, 3), R36(1, 3));
                theta6 = atan2(R36(3, 2), -R36(3, 1));
            end
            arr_theta4(i, j) = theta4;
            arr_theta6(i, j) = theta6;
        end
    end
    
    %% Generate output
    ik_sol = zeros(8, 6);
    for i = 1 : 4
        if i < 3
            idx_2row = 1;
        else
            idx_2row = 2;
        end
        theta3 = arr_theta3(idx_2row);
        theta1 = arr_theta1(idx_2row);
        
        for j = 1 : 2
            theta2 = arr_theta2(idx_2row, j);
            theta4 = arr_theta4(i, j);
            theta5 = arr_theta5(i, j);
            theta6 = arr_theta6(i, j);
            
            ik_sol(2 * i + j - 2, :) = [theta1, theta2, theta3, theta4, theta5, theta6];
        end
    end
    
end


%% helper function
function R03 = oriFrame3(theta1, theta2, theta3)
    R03 = [
        cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3),     -sin(theta1),   - cos(theta1)*cos(theta2)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)
        cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3),      cos(theta1),    - cos(theta2)*sin(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2)
        cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2),                              0,              cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)];
end