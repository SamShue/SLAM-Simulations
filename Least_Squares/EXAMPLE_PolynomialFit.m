clc;
clear all;
close all;
addpath(fullfile(fileparts(mfilename('fullpath'))));

% This extends the document's "Fitting Curves and Surfaces" section: rather
% than fitting y = c + m*t, we fit a higher order polynomial
% y = x(1) + x(2)*t + x(3)*t^2 + ... + x(degree+1)*t^degree
% by adding one more column of t.^k to A for every extra degree.
rng(0);
degree = 3;
trueCoeffs = [1; -2; 0.75; 0.2];

t = linspace(-3, 3, 30)';
noise = 0.4 * randn(size(t));
y = (t .^ (0:degree)) * trueCoeffs + noise;

% Each column k (0-indexed) of A holds t.^k -- a column of ones, a column
% of t, a column of t^2, and so on, exactly like the document's example.
A = t .^ (0:degree);

[xHat, p, e] = leastSquaresFit(A, y);

fprintf('True coefficients:\n'); disp(trueCoeffs);
fprintf('Fitted coefficients:\n'); disp(xHat);

figure; hold on; grid on;
tFit = linspace(min(t), max(t), 200)';
AFit = tFit .^ (0:degree);
plot(tFit, AFit * xHat, 'LineWidth', 2);
plot(t, y, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.95 0.75 0.1], 'MarkerEdgeColor', 'k');
xlabel('t'); ylabel('y');
title(sprintf('Least Squares Polynomial Fit (degree %d)', degree));
legend('Fitted curve', 'Data', 'Location', 'northwest');
