clear
clc
close all
warning off all
%% �������Ӳ�����Ҫ���ӵĵ�ַΪ127.0.0.1(����������)���˿ں�Ϊ5000����Ϊ�ͻ������ӡ�
Client=tcpip('127.0.0.1',5000,'NetworkRole','client');
Client.BytesAvailable
%% �������ӣ�������ɺ������һ�������򱨴�
fopen(Client);%��һ���������������ӣ�ֱ��������ɷ��أ����򱨴�
sprintf('�ɹ���������')
%% �����ַ���
sendtxt = 'hello hello';
fprintf(Client,sendtxt);
%% �����ַ���
recv3 = []; %ת����double
% pause(1);
while true
	pause(0.1)
	if Client.BytesAvailable < 256
		continue
	end
	recv=fread(Client,Client.BytesAvailable,'char');
	recv1 = dec2hex(recv);
	recv2 = [];
	for i = 1 : length(recv1)/8
		for j = 0:7
			recv2 = [recv2, recv1(8*i-j, :)];
		end
	end
	for i = 0 : length(recv1)/8-1
		recv3 = [recv3, hex2num(recv2(i*16+1:i*16+16))];
	end
end
%% �رտͻ���
fclose(Client);
