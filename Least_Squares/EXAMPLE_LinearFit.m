clc;
clear all;
close all;
addpath(fullfile(fileparts(mfilename('fullpath'))));

% This reproduces the worked example from Documents/linear_least_squares.md:
% three points that don't lie on a single line, fit with y = m*t + c.
t = [0; 1; 2];
y = [1; 2; 6];

% One column of ones (for the intercept c) and one column of t (for the
% slope m) -- the same A used throughout the document.
A = [ones(size(t)), t];

[xHat, p, e] = leastSquaresFit(A, y);

c = xHat(1);
m = xHat(2);
fprintf('Best fit line: y = %.2f*t + %.2f\n', m, c);
fprintf('Projection p:\n'); disp(p);
fprintf('Residual e:\n'); disp(e);

% Confirm the residual is orthogonal to every column of A, as the
% document's orthogonality argument predicts.
fprintf('A''*e (should be ~0):\n'); disp(A' * e);

figure; hold on; grid on;
tFit = linspace(min(t) - 0.5, max(t) + 0.5, 100);
plot(tFit, m * tFit + c, 'LineWidth', 2);
plot(t, y, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.95 0.75 0.1], 'MarkerEdgeColor', 'k');
for ii = 1:length(t)
    plot([t(ii) t(ii)], [y(ii) p(ii)], 'r--', 'LineWidth', 1.5);
end
xlabel('t'); ylabel('y');
title('Least Squares Line Fit');
legend(sprintf('y = %.2ft + %.2f', m, c), 'Data', 'Residual', 'Location', 'northwest');
