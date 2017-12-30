clear
close all

p0=[0.0; 0.0; 0.5];
xyaxes=[0 -1 0 1 0 0];
rot0 = [xyaxes(1:3)',xyaxes(4:6)',cross(xyaxes(1:3),xyaxes(4:6))'];
x0 = [0.12 0 0.0045]';

p1 = rot0*x0 + p0;
xyaxes = [0.70711 -0.70711 0 0.70711 0.70711 0];
rot1 = [xyaxes(1:3)',xyaxes(4:6)',cross(xyaxes(1:3),xyaxes(4:6))'];
x1 = [0.06068 0.04741 0]';

p2 = rot0*rot1*x1 + p1;
xyaxes = [1 0 0 0 1 0];
rot2 = [xyaxes(1:3)',xyaxes(4:6)',cross(xyaxes(1:3),xyaxes(4:6))'];
x2 = [0.43476 0.02 0]';

p3 = rot0*rot1*rot2*x2 + p2;
xyaxes = [0.52992 0.84805 0 -0.84805 0.52992 0];
rot3 = [xyaxes(1:3)',xyaxes(4:6)',cross(xyaxes(1:3),xyaxes(4:6))'];
x3 = [0.408 -0.04 0]';

p4 = rot0*rot1*rot2*rot3*x3 + p3;
x4 = [-0.01269 -0.03059 0.00092]';
p5 = rot0*rot1*rot2*rot3*x4 + p3;

x5 = [0.06068 0.08241 0]';
p6 = rot0*rot1*x5 + p1;

x6 = [0 0 0.045]';
p7 = rot0*x6 + p0;

all_points = [p0, p1, p2, p3, p4, p5, p6, p7]'