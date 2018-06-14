%Dynamic model of first 2 links of Sawyer robot

% number of links to consider
n = 2;

% DH parameters
syms a1 d1 d2 q1 q2 real

% the center of mass of each link measured relative to the link fixed frame
% (e.g. c1 = [c1x c1y c1z]' is measured relative to x1y1z1)
c = cell(n,1);
c{1} = [sym('c1x'); sym('c1y'); sym('c1z')];
c{2} = [sym('c2x'); sym('c2y'); sym('c2z')];
assume(vertcat(c{:}), 'real');

% DH tables
dh_tables = [
    -pi/2,      a1,     d1,     q1
    -pi/2,      0,      d2,     q2 - pi/2];

% Trans. matrix
T01 = homoTrans(dh_tables(1, :));
T12 = homoTrans(dh_tables(2, :));

T02 = T01 * T12;
% store trans. matrix into a cell
Ti = cell(n, 1);
Ti{1} = T01;
Ti{2} = T02;

%% Jacobian
Jv = cell(n, 1);
Jw = cell(n, 1);

% Link 1
p_0 = [0; 0; 0];
p_com_1 = T01 * [c{1}; 1];  p_com_1(end) = [];  % convert homogeneous form to cartesian form

z_0 = [0; 0; 1];

J1 = calJacobian(z_0, p_com_1, p_0); 
Jv{1} = [J1(1: 3), zeros(3, 1)];
Jw{1} = [J1(4: 6), zeros(3, 1)];

% Link 2
p_1 = T01(1:3, end);
p_com_2 = T02 * [c{2}; 1];  p_com_2(end) = [];

z_1 = T01(1:3, 3);

J2 = [calJacobian(z_0, p_com_2, p_0), calJacobian(z_1, p_com_2, p_1)];
Jv{2} = J2(1: 3, 1: end);
Jw{2} = J2(4: 6, 1: end);











