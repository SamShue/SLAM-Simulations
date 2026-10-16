function [xHat, p, e] = leastSquaresFit(A, b)
%LEASTSQUARESFIT Solve the linear least squares problem A*x = b
%   Given a coefficient matrix A and a right-hand side vector b, returns
%   the least squares solution xHat that minimizes norm(A*xHat - b), by
%   solving the normal equations A'*A*xHat = A'*b. Also returns the
%   projection p = A*xHat (the closest point to b in the column space of
%   A) and the residual error e = b - p, which is orthogonal to every
%   column of A.

    AtA = A' * A;
    Atb = A' * b;

    xHat = AtA \ Atb;
    p = A * xHat;
    e = b - p;
end
