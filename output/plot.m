clear all
close all

data = readmatrix('trajectory.txt');
x = data(:,1);
y = data(:,2);
z = data(:,3);

data1 = readmatrix('trajectory_orb_10_08.txt');
x1 = data1(:,1);
y1 = data1(:,2);
z1 = data1(:,3);

data2 = readmatrix('trajectory_orb_8_08.txt');
x2 = data2(:,1);
y2 = data2(:,2);
z2 = data2(:,3);

data3 = readmatrix('trajectory_orb_1_08.txt');
x3 = data3(:,1);
y3 = data3(:,2);
z3 = data3(:,3);

data4 = readmatrix('trajectory_orb_06_08.txt');
x4 = data4(:,1);
y4 = data4(:,2);
z4 = data4(:,3);

data5 = readmatrix('trajectory_orb_055_08.txt');
x5 = data5(:,1);
y5 = data5(:,2);
z5 = data5(:,3);

data6 = readmatrix('trajectory_orb_05_08.txt');
x6 = data6(:,1);
y6 = data6(:,2);
z6 = data6(:,3);

data7 = readmatrix('trajectory_orb_045_08.txt');
x7 = data7(:,1);
y7 = data7(:,2);
z7 = data7(:,3);

data8 = readmatrix('trajectory_orb_03_08.txt');
x8 = data8(:,1);
y8 = data8(:,2);
z8 = data8(:,3);

subplot(2, 4, 1);
plot3(x, y, z, 'b');
hold on
plot3(x1, y1, z1, 'r');
title('ORB (Ransac error = 10.0 (default))');

subplot(2, 4, 2);
plot3(x, y, z, 'b');
hold on
plot3(x2, y2, z2, 'r');
title('ORB (Ransac error = 8.0)');

subplot(2, 4, 3);
plot3(x, y, z, 'b');
hold on
plot3(x3, y3, z3, 'r');
title('ORB (Ransac error = 1.0)');

subplot(2, 4, 4);
plot3(x, y, z, 'b');
hold on
plot3(x4, y4, z4, 'r');
title('ORB (Ransac error = 0.6)');

subplot(2, 4, 5);
plot3(x, y, z, 'b');
hold on
plot3(x5, y5, z5, 'r');
title('ORB (Ransac error = 0.55)');

subplot(2, 4, 6);
plot3(x, y, z, 'b');
hold on
plot3(x6, y6, z6, 'r');
title('ORB (Ransac error = 0.5)');

subplot(2, 4, 7);
plot3(x, y, z, 'b');
hold on
plot3(x7, y7, z7, 'r');
title('ORB (Ransac error = 0.45)');

subplot(2, 4, 8);
plot3(x, y, z, 'b');
hold on
plot3(x8, y8, z8, 'r');
title('ORB (Ransac error = 0.3)');