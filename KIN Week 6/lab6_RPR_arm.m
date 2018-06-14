clc

a = 10;
b = 5;

syms theta1 d2 theta3 real

% create a new frame named frame{3p}
% origin coincide with origin of frame{2} 
% z3p coincide with z3
% x3p perpendicular to z2 and z3p (according to DH convention)
    
% DH table - last row is pose of frame{3p} relative to frame{2}
dh_table = [
    -3*pi/4,        0,        a,        theta1
    -pi/2,        0,        d2,        -pi/2
    -pi/2,        0,        0,        theta3 - 3*pi/4];
    
n = 3;  % num. joints
Ti = cell(n, 1);
for i = 1:n
    if i == 1
        Ti{i} = homoTrans(dh_table(i, :));
    else
        Ti{i} = Ti{i - 1} * homoTrans(dh_table(i, :));
    end
end
      
% pose of EE frame (frame 3) relative to frame {3p}
T3pEE = [
    0,         1,        0,        0        
    -1,        0,        0,        0        
    0,         0,        1,        b
    0,         0,        0,        1];
TEE = Ti{n} * T3pEE;
    
pos = sym(zeros(4, 3));
pos(2, :) = Ti{1}(1: 3, end)';
pos(3, :) = Ti{2}(1: 3, end)';
pos(4, :) = TEE(1: 3, end)';
  
R = TEE(1: 3, 1: 3);

%% Helper func
function T = homoTrans(dh_row)
    a = dh_row(2);
    alpha = dh_row(1);
    d = dh_row(3);
    theta = dh_row(4);
        
    T = [
        cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)
        sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)
        0, sin(alpha), cos(alpha), d
        0, 0, 0, 1]; 
end