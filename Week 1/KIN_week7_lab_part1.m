%Find the EE velocity of Lynx robot
clear all
clc
% robot param
a = 3;
b = 5.75;
c = 7.375;
d = 4.125;

% DH table
syms theta1 theta2 theta3 theta4 theta5

dh_table = [
    -pi/2       0       a       theta1
    0           b       0       theta2 - pi/2
    0           c       0       theta3 + pi/2
    -pi/2       0       0       theta4 - pi/2
    0           0       d       theta5];

%% Foward Kinematics
T01 = homoTrans(dh_table(1, :));
T12 = homoTrans(dh_table(2, :));
T23 = homoTrans(dh_table(3, :));
T34 = homoTrans(dh_table(4, :));
T45 = homoTrans(dh_table(5, :));

T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;

%% Calculate Jacobian
% Extract z and p
z_0 = [0; 0; 1];
z_1 = T01(1: 3, 3);
z_2 = T02(1: 3, 3);
z_3 = T03(1: 3, 3);
z_4 = T04(1: 3, 3);

p_0 = [0; 0; 0];
p_1 = T01(1: 3, end);
p_2 = T02(1: 3, end);
p_3 = T03(1: 3, end);
p_4 = T04(1: 3, end);
p_5 = T05(1: 3, end);

% Jacobian of each joint
J1 = calJacobian(z_0, p_5, p_0);
J2 = calJacobian(z_1, p_5, p_1);
J3 = calJacobian(z_2, p_5, p_2);
J4 = calJacobian(z_3, p_5, p_3);
J5 = calJacobian(z_4, p_5, p_4);

J = [J1, J2, J3, J4, J5];

%% End-effector velocity
arr_value_theta = [pi/2, -pi/2, pi/2, pi/3, pi/2];
arr_value_dtheta = [.1, .3, .2, -.1, .6];
ee_vel = subs(J, [theta1, theta2, theta3, theta4, theta5], arr_value_theta) * transpose(arr_value_dtheta);

ee_vel = eval(ee_vel)











