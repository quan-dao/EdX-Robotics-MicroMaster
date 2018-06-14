clear all
clc

% calculate jacobian of a R P P robot
syms d1 d2 d3 theta1

% DH table
dh_table = [
    0, 0, d1, theta1
    0, -pi/2, d2, 0
    0, 0, d3, 0];

%% transformation matrix between adjacent frames
T01 = homoTrans(dh_table(1, :));
T12 = homoTrans(dh_table(2, :));
T23 = homoTrans(dh_table(3, :));

% pose of frame i relative to frame 0
T02 = T01 * T12;
T03 = T02 * T23;

% find z and p for each frame
z_0 = [0; 0; 1];
p_0 = [0; 0; 0];

z_1 = T01(1:3, 3);
p_1 = T01(1:3, 4);

z_2 = T02(1:3, 3);
p_2 = T02(1:3, 4);

z_3 = T03(1:3, 3);
p_3 = T03(1:3, 4);

%% calculate Jacobian
J1 = calJacobian(z_0, p_3, p_0);
J2 = calJacobian(z_1);
J3 = calJacobian(z_2);

% Linear velocity Jacobian
Jv = [J1(1:3), J2(1:3), J3(1:3)]

% Angular velocity Jacobian
Jomega = [J1(4:end), J2(4:end), J3(4:end)]

% end-effector (O3) velocity
ee_vel = subs([Jv; Jomega], [d1, theta1, d2, d3], [1, -pi, 0.6, -0.3]) * [0.15; 0.1; - 0.2]