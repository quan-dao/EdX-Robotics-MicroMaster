clear all
clc
% Jacobian of standford arm

syms d2 d3 theta1 theta2 

dh_table = [
    0, -pi/2, 0, theta1
    0 pi/2, d2, theta2
    0, 0, d3, 0];

% homogeneous transformation between adjacent frame
T01 = homoTrans(dh_table(1, 1), dh_table(1, 2), dh_table(1, 3), dh_table(1, 4));
T12 = homoTrans(dh_table(2, 1), dh_table(2, 2), dh_table(2, 3), dh_table(2, 4));
T23 = homoTrans(dh_table(3, 1), dh_table(3, 2), dh_table(3, 3), dh_table(3, 4));

% pose of frame i relative frame 0
T02 = T01 * T12;
T03 = T02 * T23;

%% find the jacobian
p_3 = T03(1:3, end);

% Joint 1 - rev
z_0 = [0; 0; 1];
p_0 = zeros(3,1);
J1 = calJacobian(z_0, p_3, p_0)

% Joint 2 - rev
z_1 = T01(1:3, 3);
p_1 = T01(1:3, end);
J2 = calJacobian(z_1, p_3, p_1)

% Joint 3 - pris
z_2 = T02(1:3, 3);
J3 = calJacobian(z_2)




