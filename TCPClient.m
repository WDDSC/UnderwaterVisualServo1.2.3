clear
clc
close all
warning off all
%% 设置连接参数，要连接的地址为127.0.0.1(即本地主机)，端口号为5000，作为客户机连接。
Client=tcpip('127.0.0.1',5000,'NetworkRole','client');
Client.BytesAvailable
%% 建立连接，建立完成后进行下一步，否则报错
fopen(Client);%与一个服务器建立连接，直到建立完成返回，否则报错。
sprintf('成功建立连接')
%% 发送字符串
sendtxt = 'hello hello';
fprintf(Client,sendtxt);
%% 接收字符串
recv3 = []; %转化成double
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
%% 关闭客户端
fclose(Client);
