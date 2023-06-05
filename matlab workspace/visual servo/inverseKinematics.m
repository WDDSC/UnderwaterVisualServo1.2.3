function q = inverseKinematics(tvec, rvec)

z_base = -122.5-500;
a2 = 414;
a3 = 254;

x_obj = tvec(1);
y_obj = tvec(2);
z_obj = tvec(3);
r = sign(x_obj) * sqrt(x_obj^2+y_obj^2);
h = -z_obj+z_base;
d = sqrt(r^2+h^2);
q(1) = -atan2(y_obj, x_obj);
q(3) = -acos((d^2-a2^2-a3^2) / (2*a2*a3));
q(2) = atan(h/abs(r)) + asin(a3 * sin(pi+q(3))/d);
q(4) = 0;

if max(abs(imag(q)))>0 || max(isnan(q))>0
	q = [];
	return
end

end