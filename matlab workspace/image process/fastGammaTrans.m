close all
clear

x1 = 0; y1 = 0;
x2 = 100; y2 = 100;
x3 = 255; y3 = 255;

param = 2;
k = 1/param;

a = ((y1-y2)-k*(x1-x2)) / ((x1^2-x2^2)-2*x1*(x1-x2));
b = k-2*a*x1;
c = y1-a*x1^2-b*x1;

x = linspace(x1,x2,100);
y = a*x.^2+b*x+c;
figure()
plot([x1,x2],[y1,y2], 'b--')
hold on
plot(x,y, 'r-')
scatter([x1,x2],[y1,y2], 'r')