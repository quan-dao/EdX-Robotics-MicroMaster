function J = calJacobian(z_im1, p_n, p_im1)
%Calculate i-th column of the Jacobian 
%Input:
%z_im1: coord. of z-axis of previous frame
%p_n: coord. of E-E
%p_im1: coord. of origin of previous frame
%Output:
%6x1 matrix
%Note:
%All coord. are written in inertia frame (frame{0})
switch nargin
    case 1
        % prismatic
        J = [
            z_im1
            zeros(3,1)];
    case 3
        % revolute
        J = [
            cross(z_im1, p_n - p_im1)
            z_im1];
    otherwise
        error('Error. \nNumber of input must be either 1 (prismatic) or 3(revolute). Get %d inputs', nargin);
end

