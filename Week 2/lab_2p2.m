clear all
clc

% Description: this script calculate the inertia tensor of a cone with
% respect to its chop

% syms m h r
m = 0.1;
h = 1;
r = 0.5;

syms x y z
% declare integral dyadic
inertia_principle_moment = [
    y ^2 + z ^2
    z ^2 + x ^2
    x ^2 + y ^2];
inertia_product = [
    - x * y
    - x * z
    - y * z];
% range of z, y, x
range_z = [0, h];
R = r * z / h;  % radius at the z attitude
range_y = [-R, R];
range_x = [-sqrt(R ^2 - y ^2), sqrt(R ^2 - y ^2)];

% density
ro = 3 * m / (pi * r ^ 2 * h);

inertia_principle_moment = int(inertia_principle_moment, x, range_x);
inertia_principle_moment = int(inertia_principle_moment, y, range_y);
inertia_principle_moment = ro * int(inertia_principle_moment, z, range_z);

inertia_product = int(inertia_product, x, range_x);
inertia_product = int(inertia_product, y, range_y);
inertia_product = ro * int(inertia_product, z, range_z);

I = [
    inertia_principle_moment(1), inertia_product(1), inertia_product(2)
    inertia_product(1), inertia_principle_moment(2), inertia_product(3)
    inertia_product(2), inertia_product(3), inertia_principle_moment(3)];
