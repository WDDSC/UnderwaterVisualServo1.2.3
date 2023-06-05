clear
close all

%% 设置参数并创建机械臂
% q-d-a-alpha
DH_params = [
	0, 122.5, 0, pi/2;
	0, -195, 414, 0;
	0, 195, 0, pi/2;
	0, 254, 0, 0];
% theta_lim = [
base_mat = transl([0,0,-500])*trotx(pi/2);
% base_mat = eye(4);
theta_offset = [0, 0, -pi/2, 0];
manipulator = SerialLink(DH_params, 'name', 'manipulator');
manipulator.base = transl([0,0,-500])*trotx(180);
manipulator.offset = [0, 0, pi/2, 0];
% q1>0, 向右
% q2>0, 向下
% q3>0, 向下
% q4>0, 逆时针
q_init = [0,0,-pi,0];
q_standard = [30, 30, -30, 30]/180*pi;

% plot和teach
manipulator.plot(q_standard)
manipulator.teach(q_init);
T_d = transl([200, 10, -500]);
manipulator.ikine(T_d, [0 0 0 0], [0 0 0 0 0 0])

%% 逆运动学测试
% tvec = [200, 100, -600];
% q = inverseKinematics(tvec, []);
% manipulator.plot(q)
% hold on
% plot3(tvec(1), tvec(2), tvec(3))
% tvec_real = manipulator.fkine(q).t

%% 蒙特卡洛+逆运动学=工作空间测试
% N = 1e4;
% x = -700 + 1400 * rand(N,1);
% y = -700 + 1400 * rand(N,1);
% z = -1500 + 1800 * rand(N,1);
% tvecs = [x,y,z];
% tvecs_unreachable = [];
% tvecs_real = [];
% q = [];
% max_ik_err = 0;
% for i = 1:N
% 	q_this = inverseKinematics(tvecs(i,:), []);
% 	if isempty(q_this)
% 		tvecs_unreachable = [tvecs_unreachable; tvecs(i,:)];
% 	else
% 		q = [q; q_this];
% 		tvecs_real = [tvecs_real; manipulator.fkine(q(end,:)).t'];
% 		ik_err = sum((tvecs(i,:) - tvecs_real(end,:)).^2);
% 		if ik_err > max_ik_err
% 			max_ik_err = ik_err;
% 		end
% 	end
% end
% r = sqrt(x.^2+y.^2);
% r_real = sqrt(tvecs_real(:,1).^2 + tvecs_real(:,2).^2);
% r_unreachable = sqrt(tvecs_unreachable(:,1).^2 + tvecs_unreachable(:,2).^2);
% figure()
% scatter(r_real, tvecs_real(:,3), '*b')
% hold on
% scatter(r_unreachable, tvecs_unreachable(:,3), '.r')
% axis equal

%% 接收tvec
% tvec0;
% tvec = tvec0 + [0 0 -500];