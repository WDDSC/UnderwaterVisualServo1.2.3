clear
close all

%% 立方体目标建模
% 立方体面编号排列：1上，4前，6下，3后，2左，5右
%		1
%	2	4	5
%		6
%		3

% 1号码朝上
t(:,1) = [0 0 1]';
R(:,:,1) = [
	0	1	0
	1	0	0
	0	0	1];
T(:,:,1) = [R(:,:,1), t(:,1); [0 0 0 1]];
% 2号码朝左
t(:,2) = [0 -1 0]';
R(:,:,2) = [
	-1	0	0
	0	0	-1
	0	1	0];
T(:,:,2) = [R(:,:,2), t(:,2); [0 0 0 1]];
% 3号码朝后
t(:,3) = [1 0 0]';
R(:,:,3) = [
	0	0	1
	1	0	0
	0	-1	0];
T(:,:,3) = [R(:,:,3), t(:,3); [0 0 0 1]];
% 4号码朝前
t(:,4) = [-1 0 0]';
R(:,:,4) = [
	0	0	-1
	1	0	0
	0	1	0];
T(:,:,4) = [R(:,:,4), t(:,4); [0 0 0 1]];
% 5号码朝右
t(:,5) = [0 1 0]';
R(:,:,5) = [
	1	0	0
	0	0	1
	0	1	0];
T(:,:,5) = [R(:,:,5), t(:,5); [0 0 0 1]];
% 6号码朝下
t(:,6) = [0 0 -1]';
R(:,:,6) = [
	0	-1	0
	1	0	0
	0	0	-1];
T(:,:,6) = [R(:,:,6), t(:,6); [0 0 0 1]];

figure()

vert = [1 1 1; 1 2 1; 2 2 1; 2 1 1 ; ...
    1 1 2;1 2 2; 2 2 2;2 1 2];
vert = 2*vert - ones(8,1)*[3 3 3];
fac = [1 2 3 4; ...
    2 6 7 3; ...
    4 3 7 8; ...
    1 5 8 4; ...
    1 2 6 5; ...
    5 6 7 8];
patch('Faces',fac,'Vertices',vert,'FaceColor','c');  % patch function
% material shiny;
% alpha('color');
% alphamap('rampdown');
view(30,30);
hold on

quiver3(T(1,4,:),T(2,4,:),T(3,4,:),T(1,1,:),T(2,1,:),T(3,1,:), 'r'), hold on
quiver3(T(1,4,:),T(2,4,:),T(3,4,:),T(1,2,:),T(2,2,:),T(3,2,:), 'g')
quiver3(T(1,4,:),T(2,4,:),T(3,4,:),T(1,3,:),T(2,3,:),T(3,3,:), 'b')

xlabel('x')
ylabel('y')
zlabel('z')

axis equal

%% 
[alpha,beta,gamma,x_obj0,y_obj0,z_obj0] = textread(...
	'D:\水下机械手\视觉伺服\UnderwaterVisualServo1\WorkSpace\20220329underwater1.txt', '%f%f%f%f%f%f');
for i = 1:length(alpha)
	[n_obj0,o_obj0,a_obj0] = solvePose(alpha(i), beta(i), gamma(i));
	p_obj0(:,i) = [x_obj0(i);y_obj0(i);z_obj0(i)];
	T_obj0(:,:,i) = [n_obj0, o_obj0, a_obj0, p_obj0(:,i); [0 0 0 1]];
	[p_obj(:,i), T_obj(:,:,i)] = unifyCoordinate(T_obj0(:,:,i));
	coordinateTransMat = transl([0 0 -0.05]);
	T_objc(:,:,i) = T_obj(:,:,i) * coordinateTransMat;
	p_objc(:,i) = T_objc(1:3, 4, i);
end
figure()
plot3(p_obj(1,:), p_obj(2,:), p_obj(3,:), 'b'), hold on
plot3(p_objc(1,:), p_objc(2,:), p_objc(3,:), 'r', 'linewidth', 1.5)
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

%% 自定义函数
function [n_obj, o_obj, a_obj] = solvePose(a,b,c)
vec = [a,b,c]/norm([a,b,c]);
theta = norm([a,b,c]);
A = [0		-vec(3)	vec(2);
	vec(3)	0		-vec(1);
	-vec(2)	vec(1)	0];
RotMat = eye(3) + A*sin(theta) + A^2*(1-cos(theta));
RotMat1 = RotMat*[1 0 0; 0 0 1; 0 -1 0];
n_obj = RotMat1(1:3, 1);
o_obj = RotMat1(1:3, 2);
a_obj = RotMat1(1:3, 3);
end

function [p_obj, T_obj] = unifyCoordinate(T_obj0)
coordinateRotMat = [
	0 0 1 0
	1 0 0 0
	0 -1 0 0
	0 0 0 1];
% coordinateTransMat = transl([0 0 -0.05]);
T_obj = coordinateRotMat * T_obj0;
T_obj(1:3,4) = T_obj(1:3,4)*1000;
p_obj = T_obj(1:3, 4);
end