% Special Features: calculate derivative of a symbolic vector

% qd1 and qd2 are the time derivatives of q1 and q2 respectively
syms a1 d1 d2 q1 q2 c1 c2 c3 qd1 qd2 real

% provide your answer in terms of the above syms

syms a alp theta d 

homoTrans(a, alp, d, theta)= [
        cos(theta), -sin(theta) * cos(alp), sin(theta) * sin(alp), a * cos(theta)
        sin(theta), cos(theta) * cos(alp), -cos(theta) * sin(alp), a * sin(theta)
        0, sin(alp), cos(alp), d
        0, 0, 0, 1];
dh_table = [
    a1, -pi/2, d1, q1
    0, -pi/2, d2, q2];

T01 = homoTrans(dh_table(1,1), dh_table(1,2), dh_table(1,3), dh_table(1,4));
T12 = homoTrans(dh_table(2,1), dh_table(2,2), dh_table(2,3), dh_table(2,4));

p1 = T01 * [0; 0; c1; 1];

homo_vp1 = diff(p1, q1) * qd1;
vp1 = homo_vp1(1:3, 1)

p2 = T01 * T12 * [0; -c3; c2; 1];

homo_vp2 = diff(p2, q1) * qd1 + diff(p2, q2) * qd2;
vp2 = homo_vp2(1:3, 1)

