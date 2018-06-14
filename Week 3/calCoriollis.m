function C = calCoriollis(D, q, qd)
%Calculate the C(q, qd) in Lagrange Equations
%Input:
%D: system's inertia matrix
%q: vector of all generalized coordinates
%qd: time derivative of q

n = length(q);  % num. generalized coord.
% initialized C
C = sym(zeros(n, n));
% calculate C element-wise
for k = 1:n
   for j = 1:n
      for i = 1:n
         % calculate Christoffel symbols
         c_ijk = 0.5 * (diff(D(k, j), q(i)) + diff(D(k, i), q(j)) - diff(D(i, j), q(k)));
         C(k, j) = C(k, j) + c_ijk * qd(i);
      end
   end
end

end

