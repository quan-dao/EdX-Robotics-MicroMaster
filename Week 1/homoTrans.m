function T = homoTrans(alpha, a, d, theta)
% calculate the homogeneous transformation between frame i and frame i - 1 based on DH table
switch nargin
    case 4
        T = [
            cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)
            sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)
            0, sin(alpha), cos(alpha), d
            0, 0, 0, 1];
    case 1
        dh_row = alpha;
        % extract a, alpha, d, theta from the row of DH talbe
        a = dh_row(2);
        alpha = dh_row(1);
        d = dh_row(3);
        theta = dh_row(4);
        
        T = [
            cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)
            sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)
            0, sin(alpha), cos(alpha), d
            0, 0, 0, 1];
    otherwise
        error('Error. \nInput numbers must be 1(a row of DH table) or 4(seperated elements). Get %d', nargin);
    
end

