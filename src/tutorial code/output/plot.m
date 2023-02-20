clear all
close all

truth_trajectory = readmatrix('trajectory.txt');
x = truth_trajectory(:, 1);
y = truth_trajectory(:, 2);
z = truth_trajectory(:, 3);

sift_trajectory = readmatrix('trajectory_sift.txt');
x1 = sift_trajectory(:, 1);
y1 = sift_trajectory(:, 2);
z1 = sift_trajectory(:, 3);

[U, r, lrms] = Kabsch(truth_trajectory.', sift_trajectory.');
display(lrms);
sift_trajectory_align = sift_trajectory * U - r.';
x2 = sift_trajectory_align(:, 1);
y2 = sift_trajectory_align(:, 2);
z2 = sift_trajectory_align(:, 3);

subplot(1, 2, 1);
title('without alignment')
plot3(x, y, z, 'b');
hold on
plot3(x1, y1, z1, 'r');
hold on

subplot(1, 2, 2);
title('alignment')
plot3(x, y, z, 'b');
hold on
plot3(x2, y2, z2, 'r');
hold on
