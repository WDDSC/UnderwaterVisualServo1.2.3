close all
clear
load facets;

X = imread('test image/���Ƕ�ֱ��ͼ����1.jpg');
X = double(X);
subplot(2,2,1);imshow(uint8(X));
% colormap(map);
xlabel('(a)ԭʼͼ��');
axis square
%����������ͼ��
init = 2055615866;
randn('seed',init);
x = X + 0*randn(size(X));
subplot(2,2,2);imshow(uint8(x));
xlabel('(b)������ͼ��');
axis square
%�������ͼ���ȥ�봦��
%��С������coif3��x����2��С���ֽ�
[c,s] = wavedec2(x,2,'coif3');
%��ȡС���ֽ��е�һ��ĵ�Ƶͼ�񣬼�ʵ���˵�ͨ�˲�ȥ��
%���ó߶�����
n = [1,2];
%������ֵ����p
p = [10.12,23.28];
%�����������Ƶϵ��������ֵ����
nc = wthcoef2('h',c,s,n,p,'s');
nc = wthcoef2('v',nc,s,n,p,'s');
%���µ�С���ֽ�ṹ[c,s]�����ع�
x1 = waverec2(nc,s,'coif3');
subplot(2,2,3);imshow(uint8(x1));
xlabel('(c)��һ��ȥ��ͼ��');
axis square
%��nc�ٴν����˲�ȥ��
xx = wthcoef2('v',nc,s,n,p,'s');
x2 = waverec2(xx,s,'coif3');
subplot(2,2,4);imshow(uint8(x2));
xlabel('(d)�ڶ���ȥ��ͼ��');
axis square