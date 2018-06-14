clc
%DESC: DYNAMICS MODEL OF 2-LINK PLANAR MANIPULATOR

% robot params
% Link length: a1, a2
% COM position: ac1, ac2
% mass: m1, m2
n = 2;  % num. generalized coord.
syms a1 a2 ac1 ac2 m1 m2 real
m = cell(n, 1);
m{1} = m1;
m{2} = m2;
% generalized coord.
syms q1 q2 qd1 qd2 real
q = [
    q1
    q2];
qd = [
    qd1
    qd2];

% Jacobian
% draw from translational velocity: dot_v = Jv * dot_q
Jv = cell(n, 1);
Jv{1} = [
    -ac1 * sin(q1),     0
    ac1 * cos(q1),      0
    0,                  0];

Jv{2} = [
    -a1 * sin(q1) - ac2 * sin(q1 + q2),     -ac2 * sin(q1 + q2)
    a1 * cos(q1) + ac2 * cos(q1 + q2),      ac2 * cos(q1 + q2)];
% draw from rotational velocity: dot_w = Jw * dot_q
Jw = cell(n, 1);
Jw{1} = [
    0,      0
    0,      0
    1,      0];

Jw{2} = [
    0,      0
    0,      0
    1,      1];

% Translational part of Inertia Matrix
D_trans = zeros(n, n);
for i = 1:2
   D_trans = D_trans + m{i} * Jv{i}' * Jv{i};
end

% Inertia tensor
syms Ixy1 Izz1 Ixy2 Izz2  real
I = cell(n, 1);  % in body fixed frame, reference point = COM
I{1} = [
    0,      Ixy1,       0
    Ixy1,   0,          0
    0,      0,          Izz1];

I{2} = [
    0,      Ixy2,       0
    Ixy2,   0,          0
    0,      0,          Izz2];

% Rotational matrix
R = cell(n, 1);
R{1} = [
    cos(q1),    -sin(q1),   0
    sin(q1),    cos(q1),    0
    0,          0,          1];
R12 = [
    cos(q2),    -sin(q2),   0
    sin(q2),    cos(q2),    0
    0,          0,          1];
R{2} = R{1} * R12;

% Rotational part of Inertia Matrix
D_rot = zeros(n, n);
for i = 1:2
   D_rot = D_rot + Jw{i}'* R{i} * I{i} * R{i}' * Jw{i}; 
end

% Inertia matrix
D = D_trans + D_rot;
C = calCoriollis(D, q, qd);

% Check properties of D & C
D_dot = diff(D, q1) * qd1 + diff(D, q2) * qd2;

test_mat = D_dot - 2 * C;
skew_check = simplify(test_mat + test_mat')







